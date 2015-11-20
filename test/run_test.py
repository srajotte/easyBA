from __future__ import print_function

import os.path
import subprocess
import collections

import colorama
from termcolor import colored

easyBApath = os.path.normpath('../VisualStudio/x64/Release/easyBA.exe')
datapath = 'data'
resultpath = 'results'

dataset = [
	('bundle.out','bundle_optim.out'),
	('mono.eout','mono_optim.eout'),
	('stereo.eout','stereo_optim.eout')]

TestResults = collections.namedtuple('TestResults', ['execution', 'output_validity'])

def linecount(file):
	with open(file, 'r') as f:
		count = sum(1 for line in f)
	return count

def test_excecution(data_fullpath, result_fullpath):
	try:
		proc = subprocess.Popen([easyBApath, data_fullpath, result_fullpath], stdout=subprocess.PIPE)
	except subprocess.CalledProcessError:
		return False

	proc.communicate()
	if proc.returncode != 0:
		return False

	return True

def test_output_validity(data_fullpath, result_fullpath):
	return linecount(data_fullpath) == linecount(result_fullpath)

def run_tests(data, result):
	print('Running test with %s' % data)

	data_fullpath = os.path.join(datapath, data)
	result_fullpath = os.path.join(resultpath, result)

	test_results = []
	test_results.append(test_excecution(data_fullpath, result_fullpath))
	test_results.append(test_output_validity(data_fullpath, result_fullpath))

	return TestResults._make(test_results)

def result_str(result):
	return colored("Success!",'green') if result else colored("Fail!",'red')

def print_results(results):
	print('   Execution : %s' % result_str(results.execution))
	print('   Output Validity : %s' % result_str(results.output_validity))

def main():
	colorama.init()

	for data, result in dataset:
		test_results = run_tests(data, result)
		print_results(test_results)

if __name__ == '__main__':
	main()