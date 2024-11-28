import requests
import torch
from PIL import Image
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation
import numpy as np
import argparse
import os
from tqdm import tqdm

def create_mapillary_vistas_label_colormap():
  """Creates a label colormap used in Mapillary Vistas segmentation benchmark.

  Returns:
    A colormap for visualizing segmentation results.
  """
  return np.asarray([
      [165, 42, 42],
      [0, 192, 0],
      [196, 196, 196],
      [190, 153, 153],
      [180, 165, 180],
      [102, 102, 156],
      [102, 102, 156],
      [128, 64, 255],
      [140, 140, 200],
      [170, 170, 170],
      [250, 170, 160],
      [96, 96, 96],
      [230, 150, 140],
      [128, 64, 128],
      [110, 110, 110],
      [244, 35, 232],
      [150, 100, 100],
      [70, 70, 70],
      [150, 120, 90],
      [220, 20, 60],
      [255, 0, 0],
      [255, 0, 0],
      [255, 0, 0],
      [200, 128, 128],
      [255, 255, 255],
      [64, 170, 64],
      [128, 64, 64],
      [70, 130, 180],
      [255, 255, 255],
      [152, 251, 152],
      [107, 142, 35],
      [0, 170, 30],
      [255, 255, 128],
      [250, 0, 30],
      [0, 0, 0],
      [220, 220, 220],
      [170, 170, 170],
      [222, 40, 40],
      [100, 170, 30],
      [40, 40, 40],
      [33, 33, 33],
      [170, 170, 170],
      [0, 0, 142],
      [170, 170, 170],
      [210, 170, 100],
      [153, 153, 153],
      [128, 128, 128],
      [0, 0, 142],
      [250, 170, 30],
      [192, 192, 192],
      [220, 220, 0],
      [180, 165, 180],
      [119, 11, 32],
      [0, 0, 142],
      [0, 60, 100],
      [0, 0, 142],
      [0, 0, 90],
      [0, 0, 230],
      [0, 80, 100],
      [128, 64, 64],
      [0, 0, 110],
      [0, 0, 70],
      [0, 0, 192],
      [32, 32, 32],
      [0, 0, 0],
      [0, 0, 0],
      ])


def segmentAll(input, output):
    # load Mask2Former fine-tuned on Mapillary Vistas semantic segmentation
    processor = AutoImageProcessor.from_pretrained("facebook/mask2former-swin-large-mapillary-vistas-semantic")
    model = Mask2FormerForUniversalSegmentation.from_pretrained("facebook/mask2former-swin-large-mapillary-vistas-semantic")
    print(torch.cuda.is_available())
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    model = model.to(device)

    images = os.listdir(input)

    for imageName in tqdm(images): 
        img = Image.open(input + '/' + imageName)
        inputs = processor(images=img, return_tensors="pt").to(device)

        with torch.no_grad():
            outputs = model(**inputs)

        # model predicts class_queries_logits of shape `(batch_size, num_queries)`
        # and masks_queries_logits of shape `(batch_size, num_queries, height, width)`
        class_queries_logits = outputs.class_queries_logits
        masks_queries_logits = outputs.masks_queries_logits

        # you can pass them to processor for postprocessing
        predicted_semantic_map = processor.post_process_semantic_segmentation(outputs, target_sizes=[img.size[::-1]])[0]
        
        seg = predicted_semantic_map.to("cpu")

        # learn what everything below is doing
        color_seg = np.zeros((seg.shape[0], seg.shape[1], 3), dtype=np.uint8) # height, width, 3
        palette = np.array(create_mapillary_vistas_label_colormap())
        
        for label, color in enumerate(palette):
            color_seg[seg == label, :] = color
        # Convert to BGR
        color_seg = color_seg[..., ::-1]

        # Show image + mask
        img = np.array(img) * 0.5 + color_seg * 0.5
        img = img.astype(np.uint8)
        img = Image.fromarray(img)
        img.save(output + '/' + imageName)
        
def segmentLane(input, output):
    # load Mask2Former fine-tuned on Mapillary Vistas semantic segmentation
    processor = AutoImageProcessor.from_pretrained("facebook/mask2former-swin-large-mapillary-vistas-semantic")
    model = Mask2FormerForUniversalSegmentation.from_pretrained("facebook/mask2former-swin-large-mapillary-vistas-semantic")
    print(torch.cuda.is_available())
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    model = model.to(device)

    images = os.listdir(input)

    for imageName in tqdm(images): 
        img = Image.open(input + '/' + imageName)
        inputs = processor(images=img, return_tensors="pt").to(device)

        with torch.no_grad():
            outputs = model(**inputs)

        # Extract predicted semantic map
        predicted_semantic_map = processor.post_process_semantic_segmentation(outputs, target_sizes=[img.size[::-1]])[0]
        seg = predicted_semantic_map.to("cpu").numpy()

        # Create a binary mask for parking lot lines (class id 24), curb (class id 2)
        parking_lot_lines_mask = np.where(seg == 2, 255, 0).astype(np.uint8)
        # Save the mask
        mask_image = Image.fromarray(parking_lot_lines_mask)
        mask_filename = os.path.splitext(imageName)[0] + '.png'
        mask_image.save(os.path.join(output, mask_filename))

def main(args):
    os.makedirs(args.output, exist_ok=True)
    segmentLane(args.input, args.output)
    # segmentAll(args.input, args.output)

    # names = os.listdir(args.input)
    # print(names)

if __name__ == '__main__':
    # Create the parser and add arguments
    parser = argparse.ArgumentParser(description="Script for semantic segmenation using Mask2former trained on Mapillary Vistas from HuggingFace!")
    parser.add_argument("input", type=str, help="Input folder path to images to be segmented")
    parser.add_argument("output", type=str, help="Output folder path for segmented images")

    # Parse the arguments
    args = parser.parse_args()

    # Call the main function with the parsed arguments
    main(args)