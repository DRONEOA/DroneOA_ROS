if grep -q "FAILED" ./unittest_result.txt
then
    echo ERROR: Unittest
    cat ./unittest_result.txt
    exit 1
else
    echo PASSED: Unittest
    exit 0
fi
