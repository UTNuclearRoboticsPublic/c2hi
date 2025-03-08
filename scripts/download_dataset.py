import requests
import os
import json

from pathlib import Path

# Base URL for Dataverse API
base_url = "https://dataverse.tdl.org/api"

# A list of DOIs for all the datasets that will be downloaded
doilist = []
doilist.append("doi:10.18738/T8/KENJXS") # CHI-Ro dataset

# Define download directory
downloadbasedir = "storage/datasets"

try:
    os.mkdir(downloadbasedir)

except:
    pass


# Function to download all files from the dataset using Dataverse API
def download_dataset_files(persistent_id):
    # Construct the API endpoint for getting dataset information
    dataset_url = f"{base_url}/datasets/:persistentId/?persistentId={persistent_id}"

    # Send a GET request to the dataset API endpoint
    response = requests.get(dataset_url)

    # If the request was successful
    if response.status_code == 200:
        # Parse the JSON response
        dataset_info = response.json()

        # Check if the dataset information contains data files
        if 'data' in dataset_info and 'latestVersion' in dataset_info['data']:
            # Get the list of files from the dataset's latest version
            files = dataset_info['data']['latestVersion']['files']

        # Check if the dataset information contains data files
        if 'data' in dataset_info and 'latestVersion' in dataset_info['data']:
            # Get the list of files from the dataset's latest version
            files = dataset_info['data']['latestVersion']['files']

            # Iterate over all files
            for file_info in files:

                # Get the file ID
                file_id = file_info['dataFile']['id']

                # Get relative directory
                relative_file_path = file_info['directoryLabel']
                try:
                    full_file_path = Path(datasetdownloaddir + "/" + relative_file_path)
                    full_file_path.mkdir(parents=True, exist_ok=True)

                except OSError as error:
                    print(error)  

                # Extract the file name from the file metadata
                file_name = file_info['dataFile']['filename']

                if os.path.isfile(os.path.join(full_file_path,file_name)):
                    print("File already exists")
                else:
                    # Construct the API endpoint for downloading the file
                    file_url = f"{base_url}/access/datafile/{file_id}"

                    # Send a GET request to download the file
                    file_response = requests.get(file_url, stream=True)

                    # If the file download request was successful
                    if file_response.status_code == 200:
                        # Open a file with the same name as the downloaded file and write the content
                        with open(os.path.join(full_file_path, file_name), 'wb') as file:
                            for chunk in file_response.iter_content(chunk_size=128):
                                file.write(chunk)
                        print(f"Downloaded {file_name}")
                        file.close()
                    else:
                        print(f"Failed to download {file_name}")
        else:
            print("No files found in the dataset")
    else:
        print("Failed to access the dataset information")

# Call the function to start downloading files
for doi in doilist:

    try:
        datasetdownloaddir = downloadbasedir + "/" + doi.replace(":","-").replace(".","-").replace("/","-")
        os.mkdir(datasetdownloaddir)
        print("dataset directory made at " + datasetdownloaddir)

    except:
        pass

    try:

        try:
            metadatadir = datasetdownloaddir + "/metadata"
            os.mkdir(metadatadir)
            print("dataset metadata directory made at " + datasetdownloaddir + "/metadata")

        except:
            pass

        datasetgeneralinforequest = requests.get("https://dataverse.tdl.org/api/datasets/:persistentId/?persistentId=" + doi)
        datasetgeneralinforequest= json.loads(datasetgeneralinforequest.content.decode("latin-1"))
        datasetid = str(datasetgeneralinforequest['data']['id'])

        metadatarequest = requests.get("https://dataverse.tdl.org/api/datasets/"+datasetid+"/versions/1.0/metadata", headers={"X-Dataverse-key":config['dataverse_api_key']})
        metadata = json.loads(metadatarequest.text)

        with open(metadatadir + "/tdr-json-metadata.json","w") as metadatafile:
            json.dump(metadata, metadatafile, indent=3)

    except Exception as e:
        print(str(e))

    download_dataset_files(doi)