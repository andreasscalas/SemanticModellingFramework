#!/usr/bin/python

import os
import sys

import PhotoScan

# Ensure there is at least one argument provided.
if (len(sys.argv) < 2):
	print('No file specified.')
	sys.exit()

project_path = ''
mesh_path = ''
texture_path = ''
texture_res = 0

if (len(sys.argv) == 2):
	source_path = sys.argv[1]
	project_path = os.path.join(source_path, 'project.psx')
	mesh_path = os.path.join(source_path, 'results/final_fit_invtrans_facescan.obj')
	texture_path = os.path.join(source_path, 'textures/texture.jpg')
	texture_res = 4096

if (len(sys.argv) == 3):
	print('ERROR argv == 3')
	sys.exit()

if (len(sys.argv) == 4):
	project_path = sys.argv[1]
	mesh_path = sys.argv[2]
	texture_path = sys.argv[3]
	texture_res = 4096

if (len(sys.argv) == 5):
	project_path = sys.argv[1]
	mesh_path = sys.argv[2]
	texture_path = sys.argv[3]
	texture_res = int(sys.argv[4])

if (len(sys.argv) > 5):
	print('ERROR argv > 5')
	sys.exit()

# Initialize application and document to actually interact with PhotoScan.
app = PhotoScan.app
doc = app.document

# Look for project file and open it.
if (not os.path.isfile(project_path)):
	print('ERROR project path')
	sys.exit()
doc.open(project_path)

# Get chunk.
chunk = doc.chunk

# Look for mesh file and import it.
if (not os.path.isfile(project_path)):
	print('ERROR mesh file')
	sys.exit()
chunk.importModel(mesh_path)

# Build texture.
chunk.buildTexture(blending = PhotoScan.MosaicBlending, size = texture_res, fill_holes = True, ghosting_filter = False)

# Export texture.
chunk.model.saveTexture(texture_path)

