import numpy as np

def new_occupancies(map_new, map_old, occupancy_threshold):
    """
    Compute which points have become occupied
    """
    # Find the index of the old map origin in the new map
    origin_new = np.array((map_new.info.origin.position.x, map_new.info.origin.position.y))
    origin_old = np.array((map_old.info.origin.position.x, map_old.info.origin.position.y))
    origin_offset = origin_old - origin_new
    origin_indices = np.rint(origin_offset / map_new.info.resolution).astype(int)

    if np.any(origin_indices != 0) or \
            map_new.info.height != map_old.info.height or \
            map_new.info.width != map_old.info.width:
        # Pad the old map
        x_before = origin_indices[0]
        x_after = map_new.info.width - map_old.info.width - x_before
        y_before = origin_indices[1]
        y_after = map_new.info.height - map_old.info.height - y_before
        paddings = ((np.maximum(0, y_before),
                     np.maximum(0, y_after)),
                    (np.maximum(0, x_before),
                     np.maximum(0, x_after)))
        map_old.data = np.pad(map_old.data, paddings, 'constant', constant_values=-1)

        # Clip the old map
        x_clip_before = np.maximum(0, -x_before)
        x_clip_after = map_new.info.width + x_clip_before
        y_clip_before = np.maximum(0, -y_before)
        y_clip_after = map_new.info.height + y_clip_before
        map_old.data = map_old.data[y_clip_before:y_clip_after, x_clip_before:x_clip_after]

    # Find points that have changed to occupied
    points = np.argwhere(np.logical_and(
        map_new.data >= occupancy_threshold, 
        map_old.data < occupancy_threshold))
    points = np.fliplr(points)
    points = points * map_new.info.resolution
    points[:,0] += map_new.info.origin.position.x
    points[:,1] += map_new.info.origin.position.y

    return points
