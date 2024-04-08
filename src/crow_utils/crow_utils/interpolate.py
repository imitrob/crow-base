import argparse
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np
import pandas as pd


def get_error(xs, ys, smoothing_factor=0.5):
    spl = UnivariateSpline(xs, ys)
    spl.set_smoothing_factor(smoothing_factor)
    smoothed = spl(xs)

    err = (ys - smoothed) ** 2
    err = np.mean(err)

    return err, smoothed



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataframe", help="Dataframe in csv format")
    parser.add_argument("column", help="Name of column to interpolate")
    parser.add_argument("-i", "--index", help="Column to use as index")
    parser.add_argument("-s", "--smoothing", default=1, help="Smoothing factor to use")
    #parser.add_argument("--metric", default="mse", help="Measure of error")
    parser.add_argument("-g", "--graph", action="store_true", help="Show a graph of the data")

    args = parser.parse_args()

    #import debugpy;debugpy.listen(5678);debugpy.wait_for_client();debugpy.breakpoint();

    df = pd.read_csv(args.dataframe)

    y = df[args.column].to_numpy()
    
    if args.index:
        x = df[args.index].to_numpy()
    else:
        x = np.arange(len(df))
    
    err, smoothed = get_error(x, y, args.smoothing)
    
    print(f"mse: {err}")

    if args.graph:
        plt.plot(x, y, 'ro', ms = 5)
        plt.plot(x, smoothed)
        plt.show()

if __name__ == "__main__":
    main()