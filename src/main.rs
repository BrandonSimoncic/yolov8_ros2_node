mod yolov8;

mod yolo_model;
use opencv::prelude::*;
use opencv::Result;


use rclrs::*;
use yolo_model::YoloV8;
// use tracing::subscriber;
use std::{
    sync::{Arc, Mutex}, 
    thread, 
    time::Duration};
use sensor_msgs::msg::Image as ImageMsg;
use rclrs::log;
use builtin_interfaces::msg::Time;


fn convert_imagemsg_to_mat(image: &ImageMsg) -> Mat {
    let image_mat = Mat::from_slice(&image.data).unwrap();
    let image_mat = image_mat.reshape(3, image.height as i32).unwrap().clone_pointee();
    image_mat
}

struct Yolov8Node {
    node: Arc<Node>,
    yolov8_publisher: Arc<Publisher<ImageMsg>>,
    image_sub: Arc<Subscription<ImageMsg>>,
    data: Arc<Mutex<Option<ImageMsg>>>,
}
impl Yolov8Node {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("yolov8")?;
        let yolov8_publisher = node
            .create_publisher("yolov8_image")?;
        let data: Arc<Mutex<Option<ImageMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<ImageMsg>>> = Arc::clone(&data);
        let image_sub  = node
        .create_subscription::<ImageMsg, _>(
            "left_image",
            move |msg: ImageMsg| {
                *data_mut.lock().unwrap() = Some(msg);
            },
        )?;
        Ok(Self { node: node.into(), yolov8_publisher: yolov8_publisher.into(), image_sub: image_sub.into(), data})
    }

    fn publish_data(&self, marked_image: &Mat) -> Result<(), RclrsError> {
        let left_msg = self.construct_imagemsg(marked_image, "yolov8_image".to_string()).unwrap();
		self.yolov8_publisher.publish(left_msg).unwrap();
        Ok(())
    }
    fn construct_imagemsg(&self, image: &Mat, frame: String) -> Result<ImageMsg, RclrsError> {
        let mut msg: ImageMsg = ImageMsg::default();
		let now = self.node.get_clock().now();
        msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		msg.header.frame_id = frame;
		msg.height = image.rows() as u32;
		msg.width = image.cols() as u32;
		msg.encoding = "bgr8".to_string();
		msg.is_bigendian = 0;
		msg.step = (image.cols() * 3) as u32;
		msg.data = image.data_bytes().unwrap().to_vec();
        Ok(msg)
    }
}
fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let yolov8_node = Arc::new(Yolov8Node::new(&executor)?);
    let log_text = Arc::clone(&yolov8_node);
    let subscriber_other_thread = Arc::clone(&yolov8_node);
    let publisher_other_thread = Arc::clone(&yolov8_node);
    
    let exe_path = std::env::current_exe().unwrap();
    println!("Running executable path: {:?}", exe_path);

    // Construct the path to the .safetensors file
    // TODO: Write the get_package_share_directory function for rclrs
    let model_path = exe_path
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("share/yolov8_ros/launch/yolov8n.safetensors");
    println!("Model path: {:?}", model_path);

    // Replace args with ROS2 Params or Config file
    let cpu = false;
    let which: yolov8::Which = yolov8::Which::N;
    let con_thresh = 0.5;
    let nms = 0.5;
    let legend = 0;

    let (model, device) = yolov8::load_yolo::<YoloV8>(
        cpu,
        model_path.to_str().unwrap().to_string(), 
        which)
        .unwrap();


    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(33));
        let left_frame = publisher_other_thread.data.lock().unwrap();
        let image = convert_imagemsg_to_mat(&left_frame.as_ref().unwrap());
        log!(log_text.node.info(), "Image Size: {:?}", &image.size().unwrap());
        let yolov8_frame = yolov8::run_yolo_live(
            con_thresh, 
            nms, 
            legend, 
            image, 
            &model, 
            &device)
            .unwrap(); // TODO: This, if I sepereate this will I run into a race condition?
        log!(log_text.node.info(), "Output Size: {:?}", &yolov8_frame.size().unwrap());
        publisher_other_thread.publish_data(&yolov8_frame).unwrap();
    });
    executor.spin(SpinOptions::default()).first_error()
}
