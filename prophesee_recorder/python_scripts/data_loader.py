import os
import glob
import numpy as np


class EVADDataset:
    def __init__(self, root):
        self.root = root
        self.sensors = os.listdir(root)

        self.sensor_datasets = {}

        for sensor in self.sensors:
            path = os.path.join(self.root, sensor)
            if "imu" in sensor:
                self.sensor_datasets[sensor]= ImuDataset(path)
            elif "events" in sensor:
                self.sensor_datasets[sensor]= EventDataset(path)
            elif "trigger" in sensor:
                self.sensor_datasets[sensor]= TriggerDataset(path)


class BaseDataset:
    def __init__(self, root):
        self.data = sorted(glob.glob(os.path.join(root, "data", "*")))
        self.timestamps = np.genfromtxt(os.path.join(root, "timestamps.txt"), dtype="int64")

    def __len__(self):
        return len(self.data)


class ImuDataset(BaseDataset):
    def __getitem__(self, item):
        path = self.data[item]
        fname = os.path.basename(path).split(".")[0]
        nr = int(fname.replace("imu", ""))
        
        timestamp = self.timestamps[nr]

        imu_handle = np.load(path)
        
        imu_t = imu_handle['t'].reshape((-1,1)) + timestamp
        imu_orientation = imu_handle["orientation"]
        imu_angular_velocity = imu_handle["angular_velocity"]
        imu_linear_velocity = imu_handle["linear_velocity"]

        imu_data = np.concatenate([imu_t, imu_orientation, imu_angular_velocity, imu_linear_velocity])

        return imu_data
    

class TriggerDataset(BaseDataset):
    def __init__(self, root):
        data = np.genfromtxt(os.path.join(root, "triggers.txt"), dtype="int64")
        self.timestamps = data[:,0]
        self.polarity = data[:,-1]


class EventDataset(BaseDataset):
    def __getitem__(self, item):
        path = self.data[item]
        fname = os.path.basename(path).split(".")[0]
        nr = int(fname.replace("events", ""))
        
        timestamp = self.timestamps[nr]
        
        event_handle = np.load(path)
        e_t = event_handle["t"].reshape((-1,1)).astype("int64") + timestamp
        e_xy = event_handle["xy"].astype("int64")
        e_p = event_handle["p"].reshape((-1,1)).astype("int64")

        events = np.concatenate([e_xy, e_t, e_p], -1)
        
        return events


if __name__ == "__main__":
    data_path = "/home/dani/Documents/projects/dual_setup_calibration/data/car_setup/Bags/Prop_Sync_Static2"

    dataset = EVADDataset(data_path)

