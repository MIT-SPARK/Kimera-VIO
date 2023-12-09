# Kimera-VIO Debugging and Evaluation

Kimera-VIO comes with several tools for debugging issues in the VIO and loop-closure pipeline as well as tools for evaluating overall performance. The majority of Kimera's evaluation is handled by [`Kimera-Evaluation`](https://github.com/MIT-SPARK/Kimera-Evaluation) - this is the pipeline that is used to evaluate performance in Jenkins. However, for rapid testing the library comes with additional tools for evaluation in the form of Jupyter notebooks.

## General Debugging Tools

### Feature Tracks

### Timing

### Other

## Notebooks

Kimera-VIO comes with three notebooks for debugging and evaluating different parts of the module. You can find [plot_frontend](https://github.com/MIT-SPARK/Kimera-VIO-Evaluation/tree/master/notebooks/plot_frontend.py), [plot_backend](https://github.com/MIT-SPARK/Kimera-VIO-Evaluation/tree/master/notebooks/plot_backend.py), and [plot_lcd](https://github.com/MIT-SPARK/Kimera-VIO-Evaluation/tree/master/notebooks/plot_lcd.py) for these purposes. Each notebook is self contained and operates on the log files generated when the executable is run.

### Dependencies

To use the notebooks you will need Jupyter:

```bash
pip install jupyter
```

You will need [`evo-1`](https://github.com/MIT-SPARK/evo-1). It is currently best to build from source (consider using a virtual environment):

```bash
git clone https://github.com/MIT-SPARK/evo-1.git
cd evo-1
pip install . --upgrade --no-binary evo
```

### Usage

To use them, first run the executable with `log_output` enabled and note where the output logs will save. By default it will be in the [output_logs](output_logs/) folder. Ensure that the files are populated.

Open the notebooks and change the absolute paths near the top of the notebooks to point to the location where your output logs are saved. Then, run the notebooks. They should automatically plot relevant data, results, and statistics based on those logs.
