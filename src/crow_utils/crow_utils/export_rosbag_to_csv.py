from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import argparse
from rosbags.typesys import get_types_from_msg, register_types
import os.path as osp
import os
from pathlib import Path
from pprint import pprint
import pandas as pd
from tqdm import tqdm

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


def register_messages(folder):  # sourcery skip: dict-assign-update-to-union

    add_types = {}

    for pathstr in os.listdir(folder):
        pathstr = osp.join(folder, pathstr)
        msgpath = Path(pathstr)
        msgdef = msgpath.read_text(encoding='utf-8')
        add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))
    
    #pprint(add_types)

    register_types(add_types)


def flatten(o, name="", l=None):
    if l is None:
        l = {}

    try:
        for v, value in vars(o).items():
            if "__" not in v:
                flatten(value, f"{name + '.' if name else ''}{v}", l)
    except TypeError:
        l[name] = o

    return l


def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Path to bag folder")
    parser.add_argument("-m", "--messages", nargs="+", help="Additional message folders")
    parser.add_argument("-o", "--output", help="Location of final csv file")
    args = parser.parse_args()

    for folder in args.messages:
        register_messages(folder)

    df = None

    with Reader(args.bag) as reader:
        
        # iterate over messages
        for connection, timestamp, rawdata in tqdm(reader.messages()):
            #import debugpy;debugpy.listen(5678);debugpy.wait_for_client();debugpy.breakpoint();
            msg = deserialize_cdr(rawdata, connection.msgtype)
            flat = flatten(msg)
            flat = pd.DataFrame.from_dict([flat])

            if df is None:
                df = flat
            else:
                df = pd.concat((df, flat), ignore_index=True)

    if args.output:
        df.to_csv(args.output)
    print(df.describe())


if __name__ == "__main__":
    main()