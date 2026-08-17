find . -not -path "./build*" -a -not -path "./.git*" -name "*.[c|h]" | xargs clang-format -style=file --verbose -i 
