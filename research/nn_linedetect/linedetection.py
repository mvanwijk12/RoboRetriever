import os
from dotenv import load_dotenv, dotenv_values
import inference
import supervision as sv
import cv2

load_dotenv()

# folder = "../opencv_tests/court_lines/"
# image_file_names = ['20240820_124734.jpg','20240820_124438.jpg','20240820_124447.jpg','20240820_124451.jpg','20240820_124509.jpg','20240820_124511.jpg',
#                     '20240820_124521.jpg','20240820_124524.jpg','20240820_124535.jpg','20240820_124539.jpg','20240820_124551.jpg','20240820_124554.jpg',
#                     '20240820_124603.jpg','20240820_124612.jpg','20240820_124619.jpg','20240820_124626.jpg','20240820_124701.jpg','20240820_124708.jpg',
#                     '20240820_124732.jpg','20240820_124745.jpg','20240820_124802.jpg','20240820_124804.jpg','20240820_124806.jpg','20240820_124808.jpg',
#                     '20240820_124810.jpg','20240820_124820.jpg','20240820_124821.jpg','20240820_124826.jpg','20240820_124827.jpg','20240820_124831.jpg',
#                     '20240820_124833.jpg','20240820_124835.jpg','20240820_124838.jpg']

folder = "../opencv_tests/image_data/"
image_file_names = ['img1.png','img2.png','img3.png','img4.png','img5.png','img6.png','img7.png','img8.png','img9.png','img10.png']

current_image_index = 1

image_file = folder+image_file_names[current_image_index]
image = cv2.imread(image_file)

model = inference.get_model("roboretriver-linetest/1")
results = model.infer(image)[0]

detections = sv.Detections.from_inference(results)

bounding_box_annotator = sv.PolygonAnnotator()
label_annotator = sv.LabelAnnotator()

annotated_image = bounding_box_annotator.annotate(scene=image, detections=detections)
annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)

sv.plot_image(annotated_image)
