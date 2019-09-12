 
import os
moves = dict()
for subdir, dirs, files in os.walk("./"):
	for file in files:
        #print os.path.join(subdir, file)
		filepath = subdir + os.sep + file

		if "yz_ate_out" in filepath and "txt" in filepath:
			print(file)
			with open(filepath) as fh:
				text = fh.read().split(" ")
				rmse = str(text[4])

				move_type = subdir.split(os.sep)[1]

				move_version = file[:-4].split("_")
				move_version = move_version[-2] + "_" + move_version[-1]
				
				if not move_type in moves.keys():
					moves[move_type] = dict()
				moves[move_type][move_version] = rmse

				print(move_type, move_version, rmse)
				
