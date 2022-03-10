"""
Code that performs the segmentation of the image in order to locate the tank. However, 
the code is very complex and demands a great deal of processing power to be applied in 
real-time simulation. The results obtained with this code were made by tests with videos 
obtained from the simulation. The code was not used in the final solution
"""
import os
import torch
import torch.nn as nn
import torchvision
import torch.nn.functional as F
import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler
import torch.utils.data as data
import torchvision.models as models
import torchvision.transforms as transforms
import cv2
import numpy as np
import warnings
warnings.filterwarnings("ignore")
from collections import OrderedDict
from PIL import Image
import os

from collections import OrderedDict
import torch.utils.data as data
import numpy as np
from torchvision.transforms import ToPILImage
import torchvision.models as models
import matplotlib.pyplot as plt

class FCN32s(nn.Module):
    def __init__(self, pretrained_net, classes):
        super().__init__()
        self.classes = classes
        #self.pretrained_net = pretrained_net
        self.vgg16 = models.vgg16(pretrained=pretrained_net)
        # print(list( self.vgg16.children())) #for checking
        self.pretrained_net= list( self.vgg16.children())[0]
        self.relu    = nn.ReLU(inplace=True)
        self.deconv1 = nn.ConvTranspose2d(512, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(128)
        self.deconv2 = nn.ConvTranspose2d(128, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(128)
        self.deconv3 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(64)
        self.deconv4 = nn.ConvTranspose2d(64, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, classes, kernel_size=1)

    def forward(self, x):
        (N,C,H,W) = x.size()      
        x5 = self.pretrained_net(x)  # size=(N, 512, x.H/32, x.W/32)

        score = self.bn1(self.relu(self.deconv1(x5)))     # real size size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  #  real size size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  #  real size size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  #  real size size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  #  real size size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    #  real size size=(N, n_class, x.H/1, x.W/1)
        score =  F.interpolate(score, size=(H,W), mode='bilinear', align_corners=True) # I added this last layer to be sure about the final size

        return score  # size=(N, n_class, x.H/1, x.W/1)

model = torch.load('../Vision/model.pth', map_location='cpu')

def pil_loader(data_path, label_path):
    """Loads a sample and label image given their path as PIL images.

    Keyword arguments:
    - data_path (``string``): The filepath to the image.
    - label_path (``string``): The filepath to the ground-truth image.

    Returns the image and the label as PIL images.

    """
    data = Image.open(data_path)
    label = Image.open(label_path)

    return data, label

def batch_transform(batch, transform):
    """Applies a transform to a batch of samples.

    Keyword arguments:
    - batch (): a batch os samples
    - transform (callable): A function/transform to apply to ``batch``

    """

    # Convert the single channel label to RGB in tensor form
    # 1. torch.unbind removes the 0-dimension of "labels" and returns a tuple of
    # all slices along that dimension
    # 2. the transform is applied to each slice
    transf_slices = [transform(tensor) for tensor in torch.unbind(batch)]

    return torch.stack(transf_slices)

def remap(image, old_values, new_values):
    assert isinstance(image, Image.Image) or isinstance(
        image, np.ndarray), "image must be of type PIL.Image or numpy.ndarray"
    assert type(new_values) is tuple, "new_values must be of type tuple"
    assert type(old_values) is tuple, "old_values must be of type tuple"
    assert len(new_values) == len(
        old_values), "new_values and old_values must have the same length"

    # If image is a PIL.Image convert it to a numpy array
    if isinstance(image, Image.Image):
        image = np.array(image)

    # Replace old values by the new ones
    tmp = np.zeros_like(image)
    for old, new in zip(old_values, new_values):
        # Since tmp is already initialized as zeros we can skip new values
        # equal to 0
        if new != 0:
            tmp[image == old] = new

    return Image.fromarray(tmp)

class PILToLongTensor(object):
    """Converts a ``PIL Image`` to a ``torch.LongTensor``.
    Code adapted from: http://pytorch.org/docs/master/torchvision/transforms.html?highlight=totensor
    """

    def __call__(self, pic):
        """Performs the conversion from a ``PIL Image`` to a ``torch.LongTensor``.
        Keyword arguments:
        - pic (``PIL.Image``): the image to convert to ``torch.LongTensor``
        Returns:
        A ``torch.LongTensor``.
        """
        if not isinstance(pic, Image.Image):
            raise TypeError("pic should be PIL Image. Got {}".format(
                type(pic)))

        # handle numpy array
        if isinstance(pic, np.ndarray):
            img = torch.from_numpy(pic.transpose((2, 0, 1)))
            # backward compatibility
            return img.long()

        # Convert PIL image to ByteTensor
        img = torch.ByteTensor(torch.ByteStorage.from_buffer(pic.tobytes()))

        # Reshape tensor
        nchannel = len(pic.mode)
        img = img.view(pic.size[1], pic.size[0], nchannel)

        # Convert to long and squeeze the channels
        return img.transpose(0, 1).transpose(0,
                                             2).contiguous().long().squeeze_()

class LongTensorToRGBPIL(object):
    """Converts a ``torch.LongTensor`` to a ``PIL image``.
    The input is a ``torch.LongTensor`` where each pixel's value identifies the
    class.
    Keyword arguments:
    - rgb_encoding (``OrderedDict``): An ``OrderedDict`` that relates pixel
    values, class names, and class colors.
    """
    def __init__(self, rgb_encoding):
        self.rgb_encoding = rgb_encoding

    def __call__(self, tensor):
        """Performs the conversion from ``torch.LongTensor`` to a ``PIL image``
        Keyword arguments:
        - tensor (``torch.LongTensor``): the tensor to convert
        Returns:
        A ``PIL.Image``.
        """
        # Check if label_tensor is a LongTensor
        if not isinstance(tensor, torch.LongTensor):
            raise TypeError("label_tensor should be torch.LongTensor. Got {}"
                            .format(type(tensor)))
        # Check if encoding is a ordered dictionary
        if not isinstance(self.rgb_encoding, OrderedDict):
            raise TypeError("encoding should be an OrderedDict. Got {}".format(
                type(self.rgb_encoding)))

        # label_tensor might be an image without a channel dimension, in this
        # case unsqueeze it
        if len(tensor.size()) == 2:
            tensor.unsqueeze_(0)

        color_tensor = torch.ByteTensor(3, tensor.size(1), tensor.size(2))

        for index, (class_name, color) in enumerate(self.rgb_encoding.items()):
            # Get a mask of elements equal to index
            mask = torch.eq(tensor, index).squeeze_()
            # Fill color_tensor with corresponding colors
            for channel, color_value in enumerate(color):
                color_tensor[channel].masked_fill_(mask, color_value)

        return ToPILImage()(color_tensor)

class CamVid(data.Dataset):
    """CamVid dataset loader where the dataset is arranged as in
    https://github.com/alexgkendall/SegNet-Tutorial/tree/master/CamVid.


    Keyword arguments:
    - root_dir (``string``): Root directory path.
    - mode (``string``): The type of dataset: 'train' for training set, 'val'
    for validation set, and 'test' for test set.
    - transform (``callable``, optional): A function/transform that  takes in
    an PIL image and returns a transformed version. Default: None.
    - label_transform (``callable``, optional): A function/transform that takes
    in the target and transforms it. Default: None.
    - loader (``callable``, optional): A function to load an image given its
    path. By default ``default_loader`` is used.

    """
    # Training dataset root folders
    train_folder = 'train'
    train_lbl_folder = 'trainannot'

    # Validation dataset root folders
    val_folder = 'val'
    val_lbl_folder = 'valannot'

    # Test dataset root folders
    test_folder = 'test'
    test_lbl_folder = 'testannot'

    # Images extension
    img_extension = '.png'

    # Default encoding for pixel value, class name, and class color
    color_encoding = OrderedDict([
        ('sky', (128, 128, 128)),
        ('building', (128, 0, 0)),
        ('pole', (192, 192, 128)),
        ('road_marking', (255, 69, 0)),
        ('road', (128, 64, 128)),
        ('pavement', (60, 40, 222)),
        ('tree', (128, 128, 0)),
        ('sign_symbol', (192, 128, 128)),
        ('fence', (64, 64, 128)),
        ('car', (64, 0, 128)),
        ('pedestrian', (64, 64, 0)),
        ('bicyclist', (0, 128, 192)),
        ('unlabeled', (0, 0, 0))
    ])

    def __init__(self,
                 root_dir,
                 mode='train',
                 transform=None,
                 label_transform=None,
                 loader=pil_loader):
        self.root_dir = root_dir
        self.mode = mode
        self.transform = transform
        self.label_transform = label_transform
        self.loader = loader

        name_cond = lambda filename: "image" in filename
        ext_cond = lambda filename: filename.endswith('.png')

      
        if self.mode.lower() == 'test':
            # Get the test data and labels filepaths
            
            filtered_files = []

            # Explore the directory tree to get files that contain "name_filter" and
            # with extension "extension_filter"
            for path, _, files in os.walk("../Vision"):
                files.sort()
                for file in files:
                    if name_cond(file):
                        full_path = os.path.join(path, file)                       
                        filtered_files.append(full_path)

            
            self.test_data = filtered_files

            self.test_labels =filtered_files
    

    def __getitem__(self, index):
        """
        Args:
        - index (``int``): index of the item in the dataset

        Returns:
        A tuple of ``PIL.Image`` (image, label) where label is the ground-truth
        of the image."""

    
        if self.mode.lower() == 'test':
            data_path, label_path = self.test_data[index], self.test_labels[index]
        else:
            raise RuntimeError("Unexpected dataset mode. "
                               "Supported modes are: train, val and test")

        img, label = self.loader(data_path, label_path)

        if self.transform is not None:
            img = self.transform(img)

        if self.label_transform is not None:
            label = self.label_transform(label)

        return img, label

    def __len__(self):
        """Returns the length of the dataset."""
        if self.mode.lower() == 'train':
            return len(self.train_data)
        elif self.mode.lower() == 'val':
            return len(self.val_data)
        elif self.mode.lower() == 'test':
            return len(self.test_data)
        else:
            raise RuntimeError("Unexpected dataset mode. "
                               "Supported modes are: train, val and test")

label_transform = transforms.Compose([
        transforms.Resize((480, 640), Image.NEAREST),
        PILToLongTensor()
    ])

image_transform_test = transforms.Compose(
        [transforms.Resize((480, 640)),
         transforms.ToTensor()])

color_encoding = OrderedDict([
        ('sky', (255, 255, 255)),
        ('building', (255, 255, 255)),
        ('pole', (255, 255, 255)),
        ('road_marking', (255, 255, 255)),
        ('road', (255, 255, 255)),
        ('pavement', (255, 255, 255)),
        ('tree', (255, 255, 255)),
        ('sign_symbol', (255, 255, 255)),
        ('fence', (0, 0, 0)),
        ('car', (0, 0, 0)),
        ('pedestrian', (255, 255, 255)),
        ('bicyclist', (255, 255, 255)),
        ('unlabeled', (255, 255, 255))
    ])

test_set = CamVid(
        "../",
        mode='test',
        transform=image_transform_test,
        label_transform=label_transform)

test_loader = data.DataLoader(
        test_set,
        batch_size=1,
        shuffle=False,
        num_workers=4)

label_to_rgb = transforms.Compose([
        LongTensorToRGBPIL(color_encoding),
        transforms.ToTensor()])

m = torch.nn.Softmax2d()
model.eval()

def findTank():

    batch_images, batch_labels = iter(test_loader).next()

    with torch.no_grad():
        pred_ = model(batch_images)
        pred_ = m(pred_)

    pred = torch.empty(pred_.shape[0],480,640)

    for p in range(pred_.shape[0]):
        aux = (torch.max(pred_[p].view(pred_.shape[1],pred_.shape[2]*pred_.shape[3]), axis=0))
        pred[p] = aux.indices.view(pred_.shape[2],pred_.shape[3])

    color_predictions = batch_transform(torch.LongTensor(pred.numpy()).cpu(), label_to_rgb)
    predictionimg = color_predictions[0].numpy()

    return predictionimg
    fig, ax = plt.subplots()
    ax.imshow(np.transpose(predictionimg, (1, 2, 0)))
    fig.savefig("../Vision/filtered_image/image.png")




def convert_channels(img):
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img

def canvas_size(imgs):
    h_max = 0
    w_max = 0
    for img in imgs:
        h, w, _ = img.shape
        if h > h_max:
            h_max = h
        if w > w_max:
            w_max = w
    return w_max, h_max

def padded_imgs(imgs):
    max_w, max_h = canvas_size(imgs)
    imgs_new = []
    for img in map(convert_channels, imgs):
        h, w, _ = img.shape
        canvas = np.zeros((max_h, max_w, 3)).astype("uint8")
        canvas[:h, :w] = img
        imgs_new.append(canvas)
    return imgs_new


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('resultcnn.avi', fourcc, 20.0, (1280,480))

cap = cv2.VideoCapture('../output.avi')
if (cap.isOpened()== False):
	print("Error opening video stream or file")

while(cap.isOpened()):
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()

    if (ret):
        cv2.imwrite("../Vision/image.png", frame)
        img = findTank()*255
        img = np.dstack((img[0], img[1], img[2]))
        frame, img = padded_imgs([frame, cv2.cvtColor(img, cv2.COLOR_RGB2BGR)])
        cv2.imshow('Frame', np.hstack((img,frame)))
        cv2.waitKey(1)
        out.write(np.hstack((img,frame)))
    else:
        break

out.release()
cap.release()
cv2.destroyAllWindows()