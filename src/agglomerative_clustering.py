import numpy as np
import csv
import pandas as pd
import matplotlib.pyplot as plt
# from sklearn.cluster import KMeans, AgglomerativeCluster
from sklearn.cluster import AgglomerativeClustering
import os.path
import copy


def cal_label_count(labels):
    label_list = copy.deepcopy(labels)
    label_count = [0]*6

    for i in range(len(label_list)):
        label_count[label_list[i]] += 1
    
    return label_count
    


model_path = os.path.join("..","output","FPFH_whole_scenes_cluster","whole_r8_FPFH_r=0.3_pos.csv")
scene_path = os.path.join("..","output","FPFH_whole_scenes_cluster","PC_315966449519192000_FPFH_r=0.3_pos.csv")

initial_mean_all = pd.read_csv(model_path)
df_all = pd.read_csv(scene_path)
df_drop = df_all.dropna(axis=0,how='any')
# Cannot loaded as df_all[:,33:], should use .iloc in pd dataframe
scene_pos = df_drop.iloc[:,33:]

initial_mean = initial_mean_all.iloc[:, :33]
df = df_drop.iloc[:, :33]
print("After crop {}".format(df.shape))

# print(df)
# print(initial_mean)
mean = np.array(initial_mean)

## ward產生的群比較平均, average集中在某群
## 但兩者都能將model 6個點分到6群（predict)
cluster = AgglomerativeClustering(n_clusters=mean.shape[0], linkage = 'average')
# cluster = AgglomerativeClustering(n_clusters=mean.shape[0], linkage = 'ward')
cluster.fit(df)


np.set_printoptions(threshold=np.inf)
label_count = cal_label_count(cluster.labels_)
print(np.sum(np.array(label_count)))
print(label_count)

print(cluster.labels_)
print(type(cluster.labels_))
print("We have",cluster.labels_.shape[0],"pts.")
# print(cluster.cluster_centers_)
# print(cluster.inertia_)
bins = np.arange(0.0, mean.shape[0], 0.5)
plt.hist(cluster.labels_, bins=bins, normed=1)
plt.show()

# output_path = '/home/ee904/work/src/itri/object_detection/output/cluster.csv'
output_path = os.path.join("..","output","cluster_pos_agglomerative_average.csv")


with open(output_path, 'wb') as csvfile:
    writer = csv.writer(csvfile)
    for i in range(cluster.labels_.shape[0]):
        # row = [label, x, y, z]
        row = [cluster.labels_[i], scene_pos.iloc[i,0], scene_pos.iloc[i,1], scene_pos.iloc[i,2]] 
        # writer.writerow([cluster.labels_[i]])
        writer.writerow(row)


print(cluster.fit_predict(initial_mean))