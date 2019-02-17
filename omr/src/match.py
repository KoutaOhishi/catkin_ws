#coding: utf-8
import cv2
import numpy as np

def one(img, temp):
    # グレースケール変換
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    temp = cv2.cvtColor(temp, cv2.COLOR_RGB2GRAY)

    # テンプレート画像の高さ・幅
    h, w = temp.shape

    # テンプレートマッチング（OpenCVで実装）
    match = cv2.matchTemplate(gray, temp, cv2.TM_SQDIFF_NORMED)
    min_value, max_value, min_pt, max_pt = cv2.minMaxLoc(match)
    pt = min_pt

    # テンプレートマッチングの結果を出力
    cv2.rectangle(img, (pt[0], pt[1]), (pt[0] + w, pt[1] + h), (0,0,200), 1)
    cv2.imwrite("output.png", img)

def mul(img, temp):
    template_width = 10
    template_height = 10

    box_pt = []

    matches = cv2.matchTemplate(img, temp, cv2.TM_CCORR_NORMED)

    threshold = 0.90

    for y in xrange(matches.shape[0]):
        for x in xrange(matches.shape[1]):
            if matches[y][x] > threshold:
                cv2.rectangle(img, (x, y),
                              (x + template_width, y + template_height),
                              (0, 0, 255), 1)
                box_pt.append((x+5,y+5))


    print box_pt[0][0]

    cv2.rectangle(img, box_pt[0], box_pt[3], (255,0,0), 1)

    new=img[box_pt[0][1]:box_pt[3][1], box_pt[0][0]:box_pt[3][0]]

    cv2.imwrite("output.png", new)

def main():
    img = cv2.imread("./format.png")
    temp = cv2.imread("./tmp.png")



    #one(img, temp)
    mul(img,temp)


if __name__ == "__main__":
    main()
