import numpy as np
import csv
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

model_path = '/home/ee904/work/src/itri/object_detection/output/whole_r8_FPFH_r=0.3.csv'
scene_path = '/home/ee904/work/src/itri/object_detection/output/PC_315966449519192000_FPFH_r=0.3.csv'
initial_mean = pd.read_csv(model_path, header=None)
df = pd.read_csv(scene_path, header=None)

df_drop = df.dropna(axis=0,how='any')
# print(df_drop[1495])
print(df_drop)

print(initial_mean)

mean = np.array(initial_mean)
# print(mean)
print(mean.shape)
print(type(mean))

cluster = KMeans(n_clusters=mean.shape[0], init=mean, n_init=10)
cluster.fit(df_drop)

print(cluster.labels_)
print(type(cluster.labels_))
print(cluster.labels_.shape[0])
bins = np.arange(0.0, mean.shape[0], 0.5)
plt.hist(cluster.labels_, bins=bins)
plt.show()


output_path = '/home/ee904/work/src/itri/object_detection/output/cluster.csv'


with open(output_path, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter='\n')
    for i in range(cluster.labels_.shape[0]):
        writer.writerow(cluster.labels_[i])
