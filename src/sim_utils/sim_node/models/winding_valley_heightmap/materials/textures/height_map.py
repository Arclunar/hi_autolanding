import cv2
import numpy as np

img=cv2.imread("./heightmap_ORI.png")
img_resize= cv2.resize(img,(513,513))
print("img_resize{}".format(img_resize.shape))
# print(img)
print("hello")

gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# 显示以下图片
# cv2.imshow("gray_img",gray_img)
# cv2.waitKey(0)

noise= np.random.randn(gray_img.shape[0],gray_img.shape[1])

noise_norm = 1.5
noise = ( noise ) * 2 * noise_norm


img_with_noise = gray_img + noise
print("结果如下")
print("大小：{}".format(gray_img.shape))
print("类型：%s"%type(gray_img))
print("大小：{}".format(noise.shape))
print("类型：%s"%type(img))

print("原图")
print(gray_img)
print("噪声")
print(noise)
print("加噪声后图像")
print(img_with_noise)


# cv2.imshow("gray_img_with_noise",gray_img)
# cv2.waitKey(0)


# 保存图像
cv2.imwrite('heightmap.png', img_with_noise, [cv2.IMWRITE_PNG_COMPRESSION, 0])

