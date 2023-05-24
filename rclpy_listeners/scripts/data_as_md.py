#!/usr/bin/env python3

import pandas as pd

if __name__ == '__main__':
    fname = 'data.csv'
    df = pd.read_csv(fname)
    print(f'# Data `{fname}`')
    print(df.to_markdown())