# gesture_recognition
Gesture recognition.

Using position data from ARuco, Baxter can use the arm cameras to recognize gestures.

So far, the `gesture_recognition/action_provider` service allows for three actions: `record`, `train`, and `test`, with other optional arguments `class_label`, `filename`, and `marker_id`.

Example of recording a gesture sample:
    `rosservice call /gesture_recognition/action_provider '{action: "record", class_label: 1, marker_id: 19, filename: "~/ros_devel_ws/src/gesture_recognition/data/TrainingData.csv"}'`
    This will add a sample 1s long of position data for the marker with ID 19 with class label 1 to the class attribute trainingData and save trainingData to TrainingData.csv.

Example of training the pipeline:
    `rosservice call /gesture_recognition/action_provider '{action: "train"}'`
    This will use the stored trainingData to train the stored pipeline. If `filename` is specified, it will load the data from the file and use that to train the pipeline.

Example of testing the pipeline:
    To assess the capabilities of a combination of modules (feature extraction modules, classifiers, etc.) in the pipeline, you can run `rosservice call /gesture_recognition/action_provider '{action: "test"}'`.
    This will generate some fake data, train the stored pipeline with it, then test the pipeline's accuracy with more fake data of the same type.

The action `predict` should be able to record a sample 1s long of position data for a given marker and use the trained pipeline to print the predicted class label of the gesture. It is not working yet (5/9/17).

Eventually, the `publishGestures()` function should be able to continuously watch for gestures and publish to `gesture_recognition/result` if it has found a gesture and its predicted class label.

TODO:
* Saving and Loading CSV files using GRT capabilities does not work
* Recording a gesture sample seems buggy (first sample is always 0)
* Predict action (`predictOnce()`) does not fully work, perhaps because of AdaBoost classifier being used in the pipeline
* Continuous monitoring of gestures not fully implemented yet.
