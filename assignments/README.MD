
# Photogrammetry and Gaussian Splatting on Lunar Apollo 17 Imagery

## Photogrammetry using Agisoft Metashape

We first used **Agisoft Metashape** for 3D reconstruction. The following steps were executed in sequence:

1. **Import Images**  
   ![import images](images/agisoft_0.png)

2. **Align Images**  
   ![align images](images/agisoft_1.png)

3. **Build Model**  
   ![build model](images/agisoft_2.png)

4. **Build Texture**  
   ![build texture](images/agisoft_3.png)

5. **Export Mesh**  
   ![export mesh](images/agisoft_4.png)

Final visualization was done in **MeshLab**, with overall good quality results.

---

## Photogrammetry using COLMAP

**COLMAP** was also tested for photogrammetry. It offers more advanced parameter control but has a steeper learning curve and slightly worse results in this case.

1. **Feature Matching**  
   ![feature matching](images/colmap_0.png)

2. **Dense Reconstruction**  
   ![dense reconstruction](images/colmap_1.png)

3. **Mesh Model**  
   ![mesh model](images/colmap_2.png)

4. **Exported Mesh**  
   ![exported mesh](images/colmap_3.png)

---

## Gaussian Splatting using NeRFStudio

Gaussian splatting was done using **NeRFStudio**:

```bash
ns-process-data images --data data/Apollo/images/ --output-dir data/Apollo/ns_preprocess/
ns-train splatfacto --data data/Apollo/ns_preprocess/
ns-viewer --load-config outputs/Apollo/splatfacto/2025-04-26_155533/config.yml
ns-eval --load-config outputs/Apollo/splatfacto/2025-04-26_155533/config.yml
