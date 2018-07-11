

from shutil import copytree

# users = ['elaine', 'taylor', 'adam', 'yuchen', 'mailee', 'srinjoy', 'alex', 'priyanka', 'caleb', 'ajinkya']
# users = ['caleb']
users = ['ajinkya']
exps = ['exp2','exp3','exp4','exp5','exp6']

source_path = "/media/srinjoy/poli_ws/data_for_openface"

for user in users:
	for exp in exps:
		destination_path = "/media/psf/srinjoymajumdar/ROS_VM/data_for_{0}".format(exp)
		print "copying for user {0}".format(user)
		copy_path = "{0}/{1}/{2}".format(source_path,user,exp)
		dest_path = "{0}/{1}/{2}".format(destination_path, user, exp)
		copytree(copy_path, dest_path)
		print "finished copying for user {0}".format(user)
