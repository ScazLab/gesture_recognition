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

Eventually, the `publishGestures()` function should be able to continuously watch for gestures and publish to `gesture_recognition/result` if it has found a gesture and its predicted class label.

TODO:
* Continuous monitoring of gestures not fully implemented yet.
