import numpy as np
import csv
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import os.path

model_path = os.path.join("..","output","whole_r8_FPFH_r=0.3_pos.csv")
scene_path = os.path.join("..","output","PC_315966449519192000_FPFH_r=0.3_pos.csv")
# initial_mean = pd.read_csv(model_path, header=None)
# df = pd.read_csv(scene_path, header=None)

initial_mean_all = pd.read_csv(model_path)
df_all = pd.read_csv(scene_path)
# print(type(initial_mean_all))
# print(initial_mean_all.shape)
df_drop = df_all.dropna(axis=0,how='any')

# Cannot loaded as df_all[:,33:], should use .iloc in pd dataframe
scene_pos = df_drop.iloc[:,33:]


initial_mean = initial_mean_all.iloc[:, :33]
df = df_drop.iloc[:, :33]
print("After crop {}".format(df.shape))

print(df)
print(initial_mean)

mean = np.array(initial_mean)
# print(mean)
print(mean.shape)
# print(type(mean))

cluster = KMeans(n_clusters=mean.shape[0], init=mean, n_init=10)
cluster.fit(df)

print(cluster.labels_)
print(type(cluster.labels_))
print("We have ",cluster.labels_.shape[0],"pts.")
bins = np.arange(0.0, mean.shape[0], 0.5)
plt.hist(cluster.labels_, bins=bins)
plt.show()


# output_path = '/home/ee904/work/src/itri/object_detection/output/cluster.csv'
output_path = os.path.join("..","output","cluster_pos.csv")


with open(output_path, 'wb') as csvfile:
    writer = csv.writer(csvfile)
    for i in range(cluster.labels_.shape[0]):
        row = [cluster.labels_[i], scene_pos.iloc[i,0], scene_pos.iloc[i,1], scene_pos.iloc[i,2]] 
        # writer.writerow([cluster.labels_[i]])
        writer.writerow(row)
