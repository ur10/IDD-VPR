# IDD-VPR This is the official repo for IDD-VPR, a dataset for unstructured driving settings.


## Setup
### Conda
```bash
conda create -n seqnet numpy pytorch=1.8.0 torchvision tqdm scikit-learn faiss tensorboardx h5py -c pytorch -c conda-forge
```

### Train
To train sequential descriptors through SeqNet on the Nordland dataset:
```python
python main.py --mode train --pooling seqnet --dataset nordland-sw --seqL 10 --w 5 --outDims 4096 --expName "w5"
