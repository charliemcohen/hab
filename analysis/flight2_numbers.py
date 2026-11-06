# prints compute_numbers() as JSON
import json

import common

if __name__ == "__main__":
    print(json.dumps(common.compute_numbers(), indent=1, default=str))
