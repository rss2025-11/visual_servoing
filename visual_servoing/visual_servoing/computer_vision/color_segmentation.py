import cv2 as cv
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv.imshow("image", img)
	cv.waitKey(0)
	cv.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	#initalize
	min_x, max_x = 0,0
	min_y, max_y = 0,0
	hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

	#Find orange pixels
	# lower_orange =  np.array([5, 200, 160])
	# upper_orange = np.array([15,255,255])
	# orange_mask = cv.inRange(hsv_img, lower_orange, upper_orange)

	orange_mask = cv.inRange(hsv_img, template[0], template[1])

	# image_print(orange_mask)

	#Open img (Erode then dilate)
	kernel = cv.getStructuringElement(cv.MORPH_RECT,(5,5))
	processed_mask = cv.morphologyEx(orange_mask, cv.MORPH_OPEN, kernel)
	processed_mask = cv.morphologyEx(processed_mask, cv.MORPH_CLOSE, kernel)

	# image_print(processed_mask)

	#Counture and find bounding rect
	contours, _ = cv.findContours(processed_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

	if(len(contours) > 0):
		# cnt = contours[0]
		cnt = max(contours, key=cv.contourArea)

		x,y,w,h = cv.boundingRect(cnt)
		bounding_box = ((x,y),(x + w, y + h))

	else: 
		bounding_box = ((0,0),(0,0))
	########## YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

# def main():
# 	# orange =  np.uint8([[[0,165,255 ]]])
# 	# hsv_orange = cv.cvtColor(orange, cv.COLOR_BGR2HSV)
# 	# print(f"{hsv_orange}")
# 	lower_orange = np.array([9, 100, 125])
# 	upper_orange = np.array([11, 255, 255])
# 	image = cv.imread("./test_line_follower/line_test9.png")
# 	bbox = cd_color_segmentation(image, (lower_orange, upper_orange))

# 	print(f"bbox: {bbox[0]}, {bbox[1]}")

# 	cv.rectangle(image, bbox[0], bbox[1], (0,255,0), 3)

# 	cv.imwrite("./test_line_follower/test_output.jpg", image)

	# image_print(image)




# main()
