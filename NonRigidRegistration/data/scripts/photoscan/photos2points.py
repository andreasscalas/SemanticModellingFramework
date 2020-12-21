#!/usr/bin/python

import os
import sys
import time

import Metashape

# Ensure there is at least one argument provided.
if (len(sys.argv) < 2):
	print('No file specified.')
	sys.exit()

source_path = sys.argv[1]
source_dirs = []

# Accept either a file that contains multiple paths where to load photos from or directly a (single) path where to load photos from.
if (os.path.isfile(source_path)):
	with open(source_path) as source_file:
		source_dirs = [line.rstrip('\n') for line in source_file]
elif (os.path.isdir(source_path)):
	source_dirs.append(source_path)
else:
	print('Cannot read from ' + source_path)
	sys.exit()

background_path = ''
# The (optional) second argument is expected to point to a (single) path that holds photos of just the empty background.
if (len(sys.argv) > 2):
	path = sys.argv[2]
	if (os.path.isdir(path)):
		background_path = path
	else:
		print(path + ' is not a directory.')

# Initialize application and document to actually interact with Metashape.
app = Metashape.app
doc = app.document

# Time the beginning of putting Metashape to work.
start = time.time()

for source_dir in source_dirs:
	photo_files = []

	# Load all photos (TIFF or JPG) from the current path.
	for file in os.listdir(source_dir):
		if file.endswith('.tiff') or file.endswith('.jpg'):
			photo_files.append(os.path.join(source_dir, file))


	# Before doing anything else, clear the document and add a new chunk.
	doc.clear()
	chunk = doc.addChunk()

	# Add all photos from the current path.
	chunk.addPhotos(photo_files, strip_extensions = False)

	# If a path for background photos is provided, use those photos to create masks.
	do_background_filtering = background_path != ''
	if (do_background_filtering):
		background_files = []
		for file in os.listdir(background_path):
			if file.endswith('.jpg'):
				background_files.append(source_dir + file)

		# Ensure there is exactly one background photo for every source photo.
		if (len(photo_files) != len(background_files)):
			print('Number of source photos ({}) must equal number of background photos ({}).'.format(len(photo_files), len(background_files)))

		# Create a mask for each photo.
		for camera in chunk.cameras:
			print(camera.label)
			chunk.importMasks(os.path.join(background_path, camera.label), Metashape.MaskSourceBackground, Metashape.MaskOperationReplacement, 10, [camera])

	# Align cameras.
	camera_path = os.path.join(source_dir, 'cameras.xml')
	if (os.path.exists(camera_path)):
		chunk.importCameras(camera_path, format = Metashape.CamerasFormatXML)
		# Match photos.
		for frame in chunk.frames:
			frame.matchPhotos(accuracy = Metashape.MediumAccuracy, generic_preselection = False, reference_preselection = False, filter_mask = do_background_filtering, mask_tiepoints = False, keypoint_limit = 40000, tiepoint_limit = 4000)
		chunk.buildPoints()
	else:
		# Match photos.
		for frame in chunk.frames:
			frame.matchPhotos(accuracy = Metashape.MediumAccuracy, generic_preselection = False, reference_preselection = False, filter_mask = do_background_filtering, mask_tiepoints = False, keypoint_limit = 40000, tiepoint_limit = 4000)
		chunk.alignCameras()
		# Enlarge region to ensure ears fit inside.
		region = chunk.region
		region.size *= 2.0
		chunk.region = region

	chunk.optimizeCameras()


	# Build dense point cloud.
	chunk.buildDepthMaps(quality = Metashape.MediumQuality, filter = Metashape.AggressiveFiltering)
	chunk.buildDenseCloud(point_colors = True)

	# Export dense point cloud.
	chunk.exportPoints(os.path.join(source_dir, 'pointset.txt'), Metashape.DenseCloudData, binary = False, precision = 6, normals = True, colors = True, format = Metashape.PointsFormatXYZ)

	# Export cameras.
	chunk.exportCameras(camera_path)

	# Save project.
	doc.save(os.path.abspath(os.path.join(source_dir, 'project.psx')))

# Time when Metashape has finished.
end = time.time()
print('time taken:', end - start, 'seconds')
