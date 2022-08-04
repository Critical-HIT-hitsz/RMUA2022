import os
import cv2
import random
import numpy as np

import torch
from torch.utils import data
from torch.utils.data import Dataset

def contrast_and_brightness(img):
    alpha = random.uniform(0.75, 1.25)
    beta = random.uniform(0.75, 1.25)
    blank = np.zeros(img.shape, img.dtype)
    # dst = alpha * img + beta * blank
    dst = cv2.addWeighted(img, alpha, blank, 1-alpha, beta)
    return dst

def motion_blur(image):
    if random.randint(1,2) == 1:
        degree = random.randint(2,3)
        angle = random.uniform(-360, 360)
        image = np.array(image)
    
        # 这里生成任意角度的运动模糊kernel的矩阵， degree越大，模糊程度越高
        M = cv2.getRotationMatrix2D((degree / 2, degree / 2), angle, 1)
        motion_blur_kernel = np.diag(np.ones(degree))
        motion_blur_kernel = cv2.warpAffine(motion_blur_kernel, M, (degree, degree))
    
        motion_blur_kernel = motion_blur_kernel / degree
        blurred = cv2.filter2D(image, -1, motion_blur_kernel)
    
        # convert to uint8
        cv2.normalize(blurred, blurred, 0, 255, cv2.NORM_MINMAX)
        blurred = np.array(blurred, dtype=np.uint8)
        return blurred
    else:
        return image

def augment_hsv(img, hgain = 0.0138, sgain = 0.678, vgain = 0.36):
    r = np.random.uniform(-1, 1, 3) * [hgain, sgain, vgain] + 1  # random gains
    hue, sat, val = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    dtype = img.dtype  # uint8

    x = np.arange(0, 256, dtype=np.int16)
    lut_hue = ((x * r[0]) % 180).astype(dtype)
    lut_sat = np.clip(x * r[1], 0, 255).astype(dtype)
    lut_val = np.clip(x * r[2], 0, 255).astype(dtype)

    img_hsv = cv2.merge((cv2.LUT(hue, lut_hue), cv2.LUT(sat, lut_sat), cv2.LUT(val, lut_val))).astype(dtype)
    img = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)  # no return needed
    return img


def random_resize(img):
    h, w, _ = img.shape
    rw = int(w * random.uniform(0.8, 1))
    rh = int(h * random.uniform(0.8, 1))

    img = cv2.resize(img, (rw, rh), interpolation = cv2.INTER_LINEAR) 
    img = cv2.resize(img, (w, h), interpolation = cv2.INTER_LINEAR) 
    return img

def label_pad(x, w=640, h=640, padw=0, padh=0):
    # Convert nx4 boxes from [x, y, w, h] normalized to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    box_x = x[:,0] * w + padw
    box_y = x[:,1] * h + padh
    y[:,0] = box_x / w
    y[:,1] = box_y / h
    return y

def xy2xyxy(x, w=640, h=640):
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    centerx = x[:,0]*w
    centery = x[:,1]*h
    boxw = x[:,2]*w
    boxh = x[:,3]*h
    y[:,0] = centerx-boxw/2
    y[:,1] = centery-boxh/2
    y[:,2] = centerx+boxw/2
    y[:,3] = centery+boxh/2
    return y

def img_aug(img):
    img = contrast_and_brightness(img)
    # img = motion_blur(img)
    # img = random_resize(img)
    # img = augment_hsv(img)
    return img

def collate_fn(batch):
    img, label = zip(*batch)
    for i, l in enumerate(label):
        if l.shape[0] > 0:
            l[:, 0] = i
    return torch.stack(img), torch.cat(label, 0)

class TensorDataset():
    def __init__(self, path, img_size_width = 352, img_size_height = 352, imgaug = False):
        assert os.path.exists(path), "%s文件路径错误或不存在" % path

        self.path = path
        self.data_list = []
        self.img_size_width = img_size_width
        self.img_size_height = img_size_height
        self.mosaic_border = [-img_size_height // 4, -img_size_width // 4]
        self.img_formats = ['bmp', 'jpg', 'jpeg', 'png']
        self.imgaug = imgaug

        # 数据检查
        with open(self.path, 'r') as f:
            for line in f.readlines():
                data_path = line.strip()
                #print(data_path)
                if os.path.exists(data_path):
                    img_type = data_path.split(".")[-1]
                    if img_type not in self.img_formats:
                        raise Exception("img type error:%s" % img_type)
                    else:
                        self.data_list.append(data_path)
                else:
                    raise Exception("%s is not exist" % data_path)
        self.n = len(self.data_list)
        self.indices = range(self.n)

    def __getitem__(self, index):
        if random.random() < 0.2:
            img, labels = self.load_mosaic(index)
            # for label in labels:
            #     centerx = label[2]*self.img_size_height
            #     centery = label[3]*self.img_size_height
            #     w = label[4]*self.img_size_height
            #     h = label[5]*self.img_size_height
            #     cv2.rectangle(img, (int(centerx-w/2), int(centery-h/2)),
            #                        (int(centerx+w/2), int(centery+h/2)), (0,255,0), 2)
            # cv2.imshow('img', img);
            # cv2.waitKey(0)

            img = img.transpose(2,0,1)
        else:
            img_path = self.data_list[index]
            label_path = img_path.split(".")[0] + ".txt"

            # 归一化操作
            img = cv2.imread(img_path)
            img = cv2.resize(img, (self.img_size_width, self.img_size_height), interpolation = cv2.INTER_LINEAR) 
            #数据增强
            if self.imgaug == True:
                img = img_aug(img)
            img = img.transpose(2,0,1)

            # 加载label文件
            if os.path.exists(label_path):
                labels = []
                with open(label_path, 'r') as f:
                    for line in f.readlines():
                        l = line.strip().split(" ")
                        labels.append([0, l[0], l[1], l[2], l[3], l[4], l[5]])
                labels = np.array(labels, dtype=np.float32)

                if labels.shape[0]:
                    assert labels.shape[1] == 7, '> 5 label columns: %s' % label_path
                    #assert (label >= 0).all(), 'negative labels: %s'%label_path
                    #assert (label[:, 1:] <= 1).all(), 'non-normalized or out of bounds coordinate labels: %s'%label_path
            else:
                raise Exception("%s is not exist" % label_path) 
        
        return torch.from_numpy(img), torch.from_numpy(labels)

    def __len__(self):
        return len(self.data_list)
    
    def load_mosaic(self, index):
        labels4 = []
        s = self.img_size_height
        yc, xc = (int(random.uniform(-x, s + x)) for x in self.mosaic_border)  # mosaic center x, y
        indices = [index] + random.choices(self.indices, k=3)  # 3 additional image indices
        random.shuffle(indices)
        for i, index in enumerate(indices):
            # Load image
            img_path = self.data_list[index]
            img = cv2.imread(img_path)
            img = cv2.resize(img, (self.img_size_width, self.img_size_height), interpolation = cv2.INTER_LINEAR) 
            h, w, _ = img.shape
            img = img_aug(img)

            # place img in img4
            if i == 0:  # top left
                img4 = np.full((s, s, img.shape[2]), 114, dtype=np.uint8)  # base image with 4 tiles
                x1a, y1a, x2a, y2a = max(xc - w, 0), max(yc - h, 0), xc, yc  # xmin, ymin, xmax, ymax (large image)
                x1b, y1b, x2b, y2b = w - (x2a - x1a), h - (y2a - y1a), w, h  # xmin, ymin, xmax, ymax (small image)
            elif i == 1:  # top right
                x1a, y1a, x2a, y2a = xc, max(yc - h, 0), min(xc + w, s), yc
                x1b, y1b, x2b, y2b = 0, h - (y2a - y1a), min(w, x2a - x1a), h
            elif i == 2:  # bottom left
                x1a, y1a, x2a, y2a = max(xc - w, 0), yc, xc, min(s, yc + h)
                x1b, y1b, x2b, y2b = w - (x2a - x1a), 0, w, min(y2a - y1a, h)
            elif i == 3:  # bottom right
                x1a, y1a, x2a, y2a = xc, yc, min(xc + w, s), min(s, yc + h)
                x1b, y1b, x2b, y2b = 0, 0, min(w, x2a - x1a), min(y2a - y1a, h)

            img4[y1a:y2a, x1a:x2a] = img[y1b:y2b, x1b:x2b]  # img4[ymin:ymax, xmin:xmax]
            padw = x1a - x1b
            padh = y1a - y1b

            label_path = img_path.split(".")[0] + ".txt"
            if os.path.exists(label_path):
                label = []
                with open(label_path, 'r') as f:
                    for line in f.readlines():
                        l = line.strip().split(" ")
                        label.append([0, l[0], l[1], l[2], l[3], l[4], l[5]])
                label = np.array(label, dtype=np.float32)

                if label.shape[0]:
                    assert label.shape[1] == 7, '> 6 label columns: %s' % label_path
                    #assert (label >= 0).all(), 'negative labels: %s'%label_path
                    #assert (label[:, 1:] <= 1).all(), 'non-normalized or out of bounds coordinate labels: %s'%label_path
            else:
                raise Exception("%s is not exist" % label_path)  

            # Labels
            if label.size:
                label[:, 2:4] = label_pad(label[:, 2:4], w, h, padw, padh)  # normalized xywh to pixel xyxy format
            labels4.append(label)
        # Concat/clip labels
        labels4 = np.concatenate(labels4, 0)
        label_xyxy = labels4.copy()
        label_xyxy[:,2:6] = xy2xyxy(label_xyxy[:,2:6], self.img_size_height, self.img_size_width)
        for x in label_xyxy[:, 2:4]:
            np.clip(x, 0, s, out=x)  # clip when using random_perspective()
        row_mask = (label_xyxy[:,2:4] != 0).all(axis=1)
        labels4 = labels4[row_mask]
        label_xyxy = label_xyxy[row_mask]
        row_mask = (label_xyxy[:,2:4] != s).all(axis=1)
        labels4 = labels4[row_mask]

        return img4, labels4

if __name__ == "__main__":
    data = TensorDataset("/home/rmua6/sentry_dataset/train.txt")
    img, label = data.__getitem__(0)
    print(label.shape)
