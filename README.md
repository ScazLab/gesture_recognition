# gesture_recognition
Gesture recognition.

Using position data from ARuco, Baxter can use the arm cameras to recognize gestures.
Alternatively, set the param `use_phasespace` to true to use PhaseSpace marker data instead.

So far, the `gesture_recognition/action_provider` service allows for the following actions:
    * `record`
    * `train`
    * `predict`
    * `test`
    * `print`
with other arguments `class_label`, `filename`, and `object_id`.

Example of recording a gesture sample:
    `rosservice call /gesture_recognition/action_provider '{action: "record", class_label: 1, object_id: 19, filename: "TrainingData.csv"}'`
    This will add a sample of position data for the marker with ID 19 with class label 1 to the class attribute trainingData and save trainingData to TrainingData.csv. Files are saved to the ~/.ros directory.
    When using PhaseSpace, use `object_id` -1 to use data from both gloves, -2 for right hand only, and -3 for left hand only.

Example of training the pipeline:
    `rosservice call /gesture_recognition/action_provider '{action: "train"}'`
    This will use the stored trainingData to train the stored pipeline. If `filename` is specified, it will load the data from the file and use that to train the pipeline.

Example of testing the pipeline:
    To assess the capabilities of a combination of modules (feature extraction modules, classifiers, etc.) in the pipeline, you can run `rosservice call /gesture_recognition/action_provider '{action: "test"}'`.
    This will generate some fake data, train the stored pipeline with it, then test the pipeline's accuracy with more fake data of the same type.

Example of predicting using the trained pipeline:
    `rosservice call /gesture_recognition/action_provider '{action: "predict", object_id: 19, class_label: 1}'`
    Once the pipeline has been trained, you can use `predict` to record a gesture just like with the `record` action and then use the trained pipeline to print the predicted class label of the gesture. When `predict_and_add` is true, the gesture recording will also be added to the current trainingData, and, if `filename` is specified, the updated training dataset will be saved to `filename`.

To predict gestures continuously in real time:
    `rosservice call /gesture_recognition/action_provider '{action: "publish", object_id: -1}'`
    Once the pipeline has been trained, you can use `publish` to begin continuous recognition of gestures. This example will use both phasespace gloves as the source of marker data. Ctrl-C in the terminal where the service was called to stop continuous gesture recognition. It may be useful to call `rosbag record` in another terminal to keep track of the gesture data found and recorded during continous recognition.

Phasespace Gestures

To use the pretrained gestures recorded with the Phasespace gloves, first launch the gesture recognition program, then train using the dataset stored at "largePsTestSet.csv" (stored in ~/.ros):
`roslaunch gesture_recogntion gesture_recognition.launch`
`rosservice call /gesture_recognition/action_provider '{action: "train", filename: "largePsTestSet.csv"}'`

Now, you can use the example above to predict one gesture at a time (specify the filename to add to the recorded dataset), or use `rosservice call /gesture_recognition/action_provide '{action: "predict", object_id: -1}'` to begin continuous recognition of gestures. During continuous recognition, hold the gesture for several seconds to give the system time to record and recognize the motion.

As of April 2018, the gestures in largePsTestSet.csv are:
* 0: NULL (built in to GRT, do not record gestures of this class)
* 1: STOP (two hands with forward stopping motion)
* 2: RIGHT POINT (left hand neutral on workbench/at waist height, right hand pointing with index finger to right)
* 3: LEFT POINT (right hand neutral on workbench/at waist height, left hand pointing with index finger to left)
* 4: THUMBS UP (both hands)
* 5: RIGHT BECKON (left hand neutral, right arm starting down and out, then moving up and back towards the body, as if calling someone from far away)
* 6: LEFT BECKON (right hand neutral, left arm performing beckon described above)
* 7: RIGHT STOP (left hand neutral, right hand moving out in stopping motion to right)
* 8: LEFT STOP (right hand neutral, left hand moving out in stopping motion to left)

Demo

To use gestures to call specific actions (get, pass, hold, etc) on the robot, modify the `callAction()` function in `gesture_recognition.cpp`. As of May 2018, the gesture -> action mapping is:
* RIGHT POINT (class label 2) -> LEFT ARM GET_PASS object 155 (one of the modular furniture legs)
* LEFT POINT (class label 3) -> RIGHT ARM GET_PASS object 10 (one of the green modular furniture brackets)
* LEFT BECKON (class label 6) -> RIGHT ARM HOLD
* LEFT STOP (class label 8) -> RIGHT ARM OK/FINISHED (simulates pressing green/longer cuff button to signal completion of an action)
* ALL STOP (class label 1) -> RIGHT & LEFT ARM ERROR (simulates pressing red/round cuff button to signal the robot has made an error)

This is easily modifiable to call different actions on different arms or simulate different button presses. To run a demo with the actions described above, simply do `roslaunch gesture_recognition gesture_rec_demo.launch`. You will need to train the Gesture Rec system (`rosservice call /gesture_recognition/action_provider '{action: "train", filename: "largePsTestSet.csv"}'`) and then start publishing real-time gesture recognition (`rosservice call /gesture_recognition/action_provide '{action: "predict", object_id: -1}'`) before you begin making gestures.

Contact [Sarah Widder](sarah.widder@yale.edu) with any questions.
