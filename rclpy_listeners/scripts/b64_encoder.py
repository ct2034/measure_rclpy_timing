#!/usr/bin/env python3
"""Generate a markdown string that will encode the result images."""
import base64
import os
import traceback


for file in os.listdir():
    _, ext = os.path.splitext(file)
    if ext != ".png":
        continue
    print(f'## {file}')
    try:
        with open(file, "rb") as img_file:
            b64_string = base64.b64encode(img_file.read())
        print(f"![{file}]({b64_string.decode('utf-8')})")
    except Exception as e:
        print(f"Error:\n```\n{e}\n```")
        print(f'```\n{traceback.format_exc()}\n```')