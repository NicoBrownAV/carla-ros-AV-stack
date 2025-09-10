#!/usr/bin/env python3

import rclpy, cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO

VEH = {2,3,5,7}  # car, motorcycle, bus, truck

class CameraYOLO(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.sub = self.create_subscription(Image, '/vehicle_camera/image_raw', self.cb, 5)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections_2d', 5)
        self.pub_anno = self.create_publisher(Image, '/vehicle_camera/image_annotated', 5)
        self.score = 0.35

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        r = self.model(img, verbose=False)[0]
        out = Detection2DArray(); out.header = msg.header
        img2 = img.copy()
        for b in r.boxes:
            cls, conf = int(b.cls.item()), float(b.conf.item())
            if cls not in VEH or conf < self.score: continue
            x1,y1,x2,y2 = map(int, b.xyxy[0].tolist())
            det = Detection2D(); det.header = msg.header
            det.bbox.center.x, det.bbox.center.y = (x1+x2)/2, (y1+y2)/2
            det.bbox.size_x,  det.bbox.size_y  = (x2-x1), (y2-y1)
            hyp = ObjectHypothesisWithPose(); hyp.hypothesis.class_id=str(cls); hyp.hypothesis.score=conf
            det.results.append(hyp); out.detections.append(det)
            cv2.rectangle(img2,(x1,y1),(x2,y2),(0,255,0),2)
        self.pub_det.publish(out)
        self.pub_anno.publish(self.bridge.cv2_to_imgmsg(img2,'bgr8'))

def main():
    rclpy.init(); rclpy.spin(CameraYOLO()); rclpy.shutdown()
if __name__ == '__main__': main()

