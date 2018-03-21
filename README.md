# rqt_tf_echo
View the transform between two frames


![rqt tf echo](https://discourse-cdn-sjc2.com/standard16/uploads/ros/original/2X/4/454ef33be0f624db848a7e13856588fe8ea5885a.png)

The time of the transform and the approximate time which the transform was looked up is shown, so a transform that is not getting updated and falling behind in the buffer can be detected. In the above example the tf time is 0.0 because a static tf is being looked up.

Individual fields can be hidden with a right click context menu (though those are masked by the context menu of the line edit boxes), and whether they are hidden will be saved in .perspective files. The source and target frames are also saved and restored.

There are toolTips for each field, though I find getting them to pop up inconsistent.

The only error output is the transform time label turns red when the transform is unavailable, I’ll possibly add another field (that also can be hidden) with more verbose exception output.

TF buffer cache time will also be made configurable.

I could make the frames also have a dropdown with all the known frames, but I’d like to also make it possible to type in anything and I’m not sure if those two can be combined smoothly.
