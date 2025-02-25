mod yolov8;

mod yolo_model;
use opencv::prelude::*;
use opencv::{videoio, Result};

use rclrs::{create_node, 
    Context, 
    Node, 
    Publisher,
    Subscription,
    RclrsError, 
    QOS_PROFILE_DEFAULT,
    ToLogParams};
use tracing::subscriber;
use std::{
    env, 
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
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "yolov8").unwrap();
        let yolov8_publisher = node
            .create_publisher("left_image", QOS_PROFILE_DEFAULT)
            .unwrap();
        let data: Arc<Mutex<Option<ImageMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<ImageMsg>>> = Arc::clone(&data);
        let image_sub  = node
        .create_subscription::<ImageMsg, _>(
            "left_image",
            QOS_PROFILE_DEFAULT,
            move |msg: ImageMsg| {
                *data_mut.lock().unwrap() = Some(msg);
            },
        ).unwrap();
        Ok(Self { node, yolov8_publisher, image_sub, data})
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
    let context = Context::new(env::args()).unwrap();
    let yolov8_node = Arc::new(Yolov8Node::new(&context).unwrap());
    let log_text = Arc::clone(&yolov8_node);
    let subscriber_other_thread = Arc::clone(&yolov8_node);
    let publisher_other_thread = Arc::clone(&yolov8_node);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(33));
        
        log!(log_text.node.info(), "Subcribing to Image");
        let mut left_frame = subscriber_other_thread.data.lock().unwrap();    
    });
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(33));
        let mut left_frame = publisher_other_thread.data.lock().unwrap();
        let image = convert_imagemsg_to_mat(&left_frame.as_ref().unwrap());
        let yolov8_frame = yolov8::yolov8(&image).unwrap();
        publisher_other_thread.publish_data(&yolov8_frame).unwrap();
    });
    rclrs::spin(yolov8_node.node.clone())
}
