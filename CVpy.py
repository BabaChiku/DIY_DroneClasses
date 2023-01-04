import cv2

cap = cv2.VideoCapture(0)
detector=cv2.QRCodeDetector()

while True:
    ret, img=cap.read()
    res,bbox,img2=detector.detectAndDecode(img)
    if res:
        print(res)
        #print(bbox) #bounding box coordinate for QR code
        #print(img2) #cropped QR code out of feed RGB values
        cv2.imshow('QR crop',img2)
    if ret:
        cv2.imshow('video',img)
    if cv2.waitKey(1)==ord('q'):
        break
