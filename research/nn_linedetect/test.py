from ultralytics import YOLO
import numpy as np

# folder = "../opencv_tests/court_lines/"
# image_file_names = ['20240820_124734.jpg','20240820_124438.jpg','20240820_124447.jpg','20240820_124451.jpg','20240820_124509.jpg','20240820_124511.jpg',
#                     '20240820_124521.jpg','20240820_124524.jpg','20240820_124535.jpg','20240820_124539.jpg','20240820_124551.jpg','20240820_124554.jpg',
#                     '20240820_124603.jpg','20240820_124612.jpg','20240820_124619.jpg','20240820_124626.jpg','20240820_124701.jpg','20240820_124708.jpg',
#                     '20240820_124732.jpg','20240820_124745.jpg','20240820_124802.jpg','20240820_124804.jpg','20240820_124806.jpg','20240820_124808.jpg',
#                     '20240820_124810.jpg','20240820_124820.jpg','20240820_124821.jpg','20240820_124826.jpg','20240820_124827.jpg','20240820_124831.jpg',
#                     '20240820_124833.jpg','20240820_124835.jpg','20240820_124838.jpg']

# folder = "../opencv_tests/image_data/"
# image_file_names = ['img1.png','img2.png','img3.png','img4.png','img5.png','img6.png','img7.png','img8.png','img9.png','img10.png']

folder = "../opencv_tests/new_images/"
image_file_names = ['image_1-2024-09-13_14-08-15.jpg','image_2-2024-09-13_14-08-15.jpg','image_3-2024-09-13_14-08-15.jpg','image_4-2024-09-13_14-08-15.jpg',
                    'image_5-2024-09-13_14-08-15.jpg','image_6-2024-09-13_14-08-15.jpg','image_7-2024-09-13_14-08-15.jpg','image_8-2024-09-13_14-08-15.jpg',
                    'image_9-2024-09-13_14-08-15.jpg','image_10-2024-09-13_14-08-15.jpg','image_11-2024-09-13_14-08-15.jpg','image_12-2024-09-13_14-08-15.jpg',
                    'image_13-2024-09-13_14-08-15.jpg','image_14-2024-09-13_14-08-15.jpg','image_15-2024-09-13_14-08-15.jpg','image_16-2024-09-13_14-08-15.jpg',
                    'image_17-2024-09-13_14-08-15.jpg','image_18-2024-09-13_14-08-15.jpg','image_19-2024-09-13_14-08-15.jpg','image_20-2024-09-13_14-08-15.jpg']

current_image_index = 19  # 0 or 2

model = YOLO('best.pt')
results = model([folder+image_file_names[current_image_index]])

for result in results:
    boxes = result.boxes
    masks = result.masks
    #print(masks.shape)  # torch.Size([1, 480, 640])

    mask = np.array(masks[0].data)
    #print(mask)

    print(np.sum(mask==0))
    print(np.sum(mask>0))

    result.show()  # display to screen

# print(results)

# https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Boxes.xyxyn

