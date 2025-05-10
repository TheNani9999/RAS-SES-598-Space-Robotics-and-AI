
# Photogrammetry and Gaussian Splatting on Lunar Apollo 17 Imagery

## Photogrammetry using Agisoft Metashape

We first used **Agisoft Metashape** for 3D reconstruction. The following steps were executed in sequence:

1. **Import Images**  
  ![AS4 1](https://github.com/user-attachments/assets/c10d6ead-88ee-41e0-ae88-e402027d6947)


2. **Align Images**  

  
![AS4 2](https://github.com/user-attachments/assets/a5569c39-c5a7-4bac-afcc-64f5b0655b9c)

![AS4 3 ALIGN ](https://github.com/user-attachments/assets/4a076a35-ebd9-4068-b3f5-79dd667f7a6b)


3. **Build Model**  
![AS4 4 MODEL BUILD](https://github.com/user-attachments/assets/44e75117-ff50-4597-992f-0fbeb301b018)


![AS4 5 MB](https://github.com/user-attachments/assets/d342ae4f-2611-4324-bc9e-1a8ec543bd41)


5. **Build Texture**  
![AS4 6 TEXTURE](https://github.com/user-attachments/assets/5242f487-967b-4a47-9c66-3a39325e6f48)

![AS4 7](https://github.com/user-attachments/assets/4f1d03d1-fad7-4008-8c69-b520f418e9b3)


## Photogrammetry using COLMAP

**COLMAP** was also tested for photogrammetry. It offers more advanced parameter control but has a steeper learning curve and slightly worse results in this case.

### feature matching colmap
![AS4 8 COLMAP](https://github.com/user-attachments/assets/e59c110b-2df6-4045-882b-e10004831f48)

![AS4 9 COLMAP](https://github.com/user-attachments/assets/84570b2f-1b80-4c9d-877e-89fc17bae257)

### dense reconstruction colmap_1
https://github.com/user-attachments/assets/55b195a3-46be-400c-93b8-794833937956

## Gaussian Splatting using NeRFStudio

Gaussian splatting was performed using NeRFStudio, with the training process taking approximately 45 minutes!

https://github.com/user-attachments/assets/da62617e-ed4f-466f-9cbd-7ec9fc12428e


{
  "experiment_name": "Apollo",
  "method_name": "splatfacto",
  "checkpoint": "outputs/Apollo/splatfacto/2025-05-08_141200/nerfstudio_models/step-000025000.ckpt",
  "results": {
    "psnr": 16.34219264984131,
    "psnr_std": null,
    "ssim": 0.2381298542022705,
    "ssim_std": null,
    "lpips": 0.6810271730422974,
    "lpips_std": null,
    "num_rays_per_sec": 712345.25,
    "num_rays_per_sec_std": null,
    "fps": 0.5193849201202393,
    "fps_std": null
  }
}



![Results](https://github.com/user-attachments/assets/53b881ed-4b67-4ee7-882d-95fa4e096263)

From a quantitative standpoint, the final results were: PSNR = 16.342, SSIM = 0.238, and LPIPS = 0.681.
PSNR (Peak Signal-to-Noise Ratio) of 16.342 suggests a suboptimal reconstruction quality.
SSIM (Structural Similarity Index) at 0.238 indicates limited structural preservation.
LPIPS (Learned Perceptual Image Patch Similarity) of 0.681 reveals significant perceptual differences between the synthesized and reference views.

# Photogrammetry with Augmented Views (N=25)
![N25](https://github.com/user-attachments/assets/41def5a2-36f6-4d62-a81d-e76b58828d7d)

# Quantitative Evaluation of Results
To assess the quality of the reconstructed models, we employed both 3D and 2D evaluation methods, given the absence of a reference ground truth model.

## 3D Model Evaluation:
In the absence of a definitive ground truth mesh, we relied on indirect metrics such as mesh topology complexity and geometric density, including the total number of vertices and faces. A higher count often reflects greater detail, although this does not always correlate with better accuracy. Other possible metrics include surface smoothness, hole count, and normal consistency, which help evaluate model fidelity.

## 2D Image Evaluation:
To determine how well the reconstructed model matches the input imagery, we calculated the texture reprojection error. This measures the discrepancy between the original image projections and the rendered model views. Lower errors suggest that the model better preserves visual and spatial information. Additional evaluations may involve comparing image sharpness, color consistency, and feature alignment across viewpoints.

## Software-Aided Alignment and Comparison:
Tools like CloudCompare were used to align the original and enhanced models for direct comparison. This allows for a visual and numerical inspection of improvements or degradations across metrics such as point-to-point distance, change in surface area, or density distribution.



