# Download models from the SubT Tech Repo into .zip files in the current directory (does not require Ignition install)
# May choose a subset of models (e.g., robots, artifacts, tiles) or download all models
#
#   Usage: 
#           python download_models.py <TYPE>
#
#   Valid types:
#           1: All models
#           2: Robots
#           3: Artifacts 
#           4: Tunnel tiles
#           5: Urban tiles
#           6: Cave tiles

import sys, json, requests

model_filters = {
    1: "*",
    2: "categories:Robots",
    3: "tags:artifact",
    4: "Tunnel*",
    5: "Urban*",
    6: "Cave*"
}

if (len(sys.argv) != 2):
    print("""Usage: python download_models.py <TYPE>
    Valid types:
        1: All models
        2: Robots
        3: Artifacts 
        4: Tunnel tiles
        5: Urban tiles
        6: Cave tiles""")
    exit()
else:
    model_filter = model_filters.get(int(sys.argv[1]), "*") 
    # Note: if the type is not in the list, all models will be downloaded
    print("Downloading SubT Tech Repo models matching the filter: %s\nThis may take a few minutes..." % model_filter)

# URLs for getting model names and files
repo_url = 'https://fuel.ignitionrobotics.org/1.0/models?per_page=500&q=collections:SubT%20Tech%20Repo%26'
download_url = 'https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/'

# Get a list of models matching the filter
models = requests.get(repo_url+model_filter)
models_dict = json.loads(models.text)

# For each model in list, download the model
for entry in models_dict:
    model_name = entry['name']
    print('  Downloading %s' % model_name)
    download_res = requests.get(download_url+model_name+'.zip',stream=True)
    with open(model_name+'.zip', 'wb') as fd:
        for chunk in download_res.iter_content(chunk_size=1024*1024):
            fd.write(chunk)
print('Done.')
