#/bin/bash

find . -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i --sort-includes {} \;
