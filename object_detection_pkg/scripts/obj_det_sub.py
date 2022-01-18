#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 

class ObjectDetector():
    def __init__(self):
        config_path = 'src/object_detection_pkg/scripts/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        weights_path = 'src/object_detection_pkg/scripts/frozen_inference_graph.pb'
        class_file = 'src/object_detection_pkg/scripts/coco.names'
        with open(class_file, 'rt') as f:
            self.class_names = f.read().rstrip('\n').split('\n')

        self.net = cv2.dnn_DetectionModel(weights_path, config_path)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)
        
        self.thresh = 0.6
        self.nms_thresh = 0.4

    def infer(self, frame):
        og_shp = frame.shape
        frame = cv2.resize(frame, (640, 480))
        frame = cv2.flip(frame, 1)
        class_ids, confs, bboxes = self.net.detect(frame, confThreshold=self.thresh)
        if len(class_ids) != 0:
            bboxes = list(bboxes)
            confs = list(confs.reshape(1, -1)[0])
            confs = list(map(float, confs))
            indices = cv2.dnn.NMSBoxes(bboxes, confs, self.thresh, self.nms_thresh)
            
            for i in indices:
                bbox = bboxes[i]
                x, y, w, h = bbox[0], bbox[1], bbox[2], bbox[3]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)
                cv2.putText(frame, f'{self.class_names[class_ids[i] - 1]}({round(confs[i] * 100, 2)})',
                            (bbox[0] + 10, bbox[1] + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                
        frame = cv2.resize(frame, (og_shp[1], og_shp[0]))
        return frame

    def callback(self, msg):
        br = CvBridge()
        rospy.loginfo("received frame")
        curr_frame = br.imgmsg_to_cv2(msg)
        obj_det_out = self.infer(curr_frame)
        cv2.imshow("output", obj_det_out)
        cv2.waitKey(1)

def receive_message():
    rospy.init_node('obj_det_sub', anonymous=True)
    object_detector = ObjectDetector()
    rospy.Subscriber('webcam_feed', Image, object_detector.callback)
    rospy.spin()
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()
