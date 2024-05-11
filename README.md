# IDD-VPR
This is the official repo for IDD-VPR, a dataset for unstructured driving settings.


## Setup
### Conda
```bash
conda create -n idd-vpr numpy pytorch=1.8.0 torchvision tqdm scikit-learn faiss tensorboardx h5py -c pytorch -c conda-forge
```

### Train
To train sequential descriptors through SeqNet on the Nordland dataset:
```python
python main.py --mode train --model MixVPR --dataset IDD-VPR --challenge Illumination --outDims 4096 --expName "w5"

### Test
On the IDD-VPR dataset
```python
python main.py --mode test --pooling MixVPR --dataset IDD-VPR --outDims 4096

On the Nordland dataset:
```python
python main.py --mode test --pooling seqnet --dataset nordland-sf --seqL 5 --split test --resume ./data/runs/Jun03_15-22-44_l10_w5/ 
```
On the MSLS dataset (can change `--msls_valCity` to `melbourne` or `austin` too):
```python
python main.py --mode test --pooling seqnet --dataset msls --msls_valCity amman --seqL 5 --split test --resume ./data/runs/<modelName>/
```


```
</details>

### Single Image Vanilla NetVLAD Extraction
<details>
<summary> [Expand this] To obtain the single image vanilla NetVLAD descriptors (i.e. the provided precomputed .npy descriptors) </summary>

```bash
# Setup Patch-NetVLAD submodule from the seqNet repo:
cd seqNet 
git submodule update --init

# Download NetVLAD+PCA model
cd thirdparty/Patch-NetVLAD/patchnetvlad/pretrained_models
wget -O pitts_orig_WPCA4096.pth.tar https://cloudstor.aarnet.edu.au/plus/s/gJZvogRj4FUUQMy/download

# Compute global descriptors
cd ../../../Patch-NetVLAD/
python feature_extract.py --config_path patchnetvlad/configs/seqnet.ini --dataset_file_path ../../structFiles/imageNamesFiles/oxford_2014-12-16-18-44-24_imagenames_subsampled-2m.txt --dataset_root_dir <PATH_TO_OXFORD_IMAGE_DIR> --output_features_fullpath ../../data/descData/netvlad-pytorch/oxford_2014-12-16-18-44-24_stereo_left.npy

# example for MSLS (replace 'database' with 'query' and use different city names to compute all)
python feature_extract.py --config_path patchnetvlad/configs/seqnet.ini --dataset_file_path ../../structFiles/imageNamesFiles/msls_melbourne_database_imageNames.txt --dataset_root_dir <PATH_TO_Mapillary_Street_Level_Sequences> --output_features_fullpath ../../data/descData/netvlad-pytorch/msls_melbourne_database.npy
```
</details>
  
