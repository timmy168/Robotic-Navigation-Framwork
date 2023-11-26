import cv2

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Pixel coordinates (x, y): ({x}, {y})")

if __name__ == "__main__":
    image_path = "map.png"

    # read the image
    img = cv2.imread(image_path)

    cv2.namedWindow("image")
    cv2.setMouseCallback("image", click_event)

    while True:
        # Show the image
        cv2.imshow("image", img)

        # Press Esc to quit
        if cv2.waitKey(20) & 0xFF == 27:
            break

    cv2.destroyAllWindows()