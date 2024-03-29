#!/bin/bash -x
function main(){
    local -r PY_FILES=`find . -name "*.py"`
    for x in ${PY_FILES}
    do
        autoflake --remove-all-unused-imports --in-place ${x}
    done
}

main "$@"
