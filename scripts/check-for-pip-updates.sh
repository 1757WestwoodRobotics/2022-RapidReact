#!/bin/bash

OUTPUT=$(pip list --outdated --exclude=setuptools)
if [[ ! -z "$OUTPUT" ]]
then
    echo "$OUTPUT"
    exit 1
else
    echo "pip packages are up to date"
fi

exit 0
