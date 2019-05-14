# manipulation actions
Standalone manipulation actions and testing for the FetchIt! challenge.

## available action servers for the task controller
* **in-hand localization**
  * topic: "in_hand_localizer/localize"
  * message: manipulation_actions/InHandLocalizeAction
  * pre-reqs: robot is holding on object
* **place object in bin**
  * topic: "placer/store_object"
  * message: manipulation_actions/StoreObjectAction
  * pre-reqs: the active bin has been detected, in-hand localization has been run
* **pick screw from bowl**
  * topic: "cluttered_grasper/blind_bin_pick"
  * message: manipulation_actions/BinPickAction
  * pre-reqs: rail_segmentation was run on the screw bin
* **schunk insertion**
  * topic: "schunk_insert"
  * message: manipulation_actions/SchunkInsertAction
  * pre-reqs: robot is holding big gear and big-gear tf x-axis is aligned to schunk machine x-axis
