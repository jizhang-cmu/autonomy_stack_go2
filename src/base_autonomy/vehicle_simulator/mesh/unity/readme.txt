Environment files are in this folder.

'environment' contains the Unity environment model. In the folder, 'AssetList.csv' is generated at runtime containing all assets in the environment model and their poses and semantic rendering colors, 'Dimensions.csv' and 'Categories.csv' contain the dimension information and category mapping.

'map.ply' is the point cloud of the environment in binary format.

'object_list.txt' is the object list with bounding boxes and labels of each object. Each row has 'object_id, x (m), y (m), z (m), length (m), width (m), height (m), orientatoin_of_length_edge (rad), label (string)'.

'traversable_area.ply' is the traversable area for the robot.

'map.jpg' is a screenshot of the segmented point cloud.

'render.jpg' is an image render of the environment.
