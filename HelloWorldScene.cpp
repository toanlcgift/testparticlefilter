/****************************************************************************
 Copyright (c) 2017-2018 Xiamen Yaji Software Co., Ltd.

 http://www.cocos2d-x.org

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/

#include "HelloWorldScene.h"
#include "SimpleAudioEngine.h"
#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>

#include "particle_filter.h"
#include "helper_functions.h"
using namespace std;
USING_NS_CC;

Scene* HelloWorld::createScene()
{
	return HelloWorld::create();
}

// Print useful error message instead of segfaulting when files are not there.
static void problemLoading(const char* filename)
{
	printf("Error while loading: %s\n", filename);
	printf("Depending on how you compiled you might have to add 'Resources/' in front of filenames in HelloWorldScene.cpp\n");
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
	//////////////////////////////
	// 1. super init first
	if (!Scene::init())
	{
		return false;
	}

	auto visibleSize = Director::getInstance()->getVisibleSize();
	Vec2 origin = Director::getInstance()->getVisibleOrigin();

	/////////////////////////////
	// 2. add a menu item with "X" image, which is clicked to quit the program
	//    you may modify it.

	// add a "close" icon to exit the progress. it's an autorelease object
	auto closeItem = MenuItemImage::create(
		"CloseNormal.png",
		"CloseSelected.png",
		CC_CALLBACK_1(HelloWorld::menuCloseCallback, this));

	if (closeItem == nullptr ||
		closeItem->getContentSize().width <= 0 ||
		closeItem->getContentSize().height <= 0)
	{
		problemLoading("'CloseNormal.png' and 'CloseSelected.png'");
	}
	else
	{
		float x = origin.x + visibleSize.width - closeItem->getContentSize().width / 2;
		float y = origin.y + closeItem->getContentSize().height / 2;
		closeItem->setPosition(Vec2(x, y));
	}

	// create menu, it's an autorelease object
	auto menu = Menu::create(closeItem, NULL);
	menu->setPosition(Vec2::ZERO);
	this->addChild(menu, 1);
	InitValue();
	return true;
}

void HelloWorld::InitValue() {
	// parameters related to grading.
	int time_steps_before_lock_required = 100; // number of time steps before accuracy is checked by grader.
	double max_runtime = 45; // max allowable runtime to pass [sec]
	double max_translation_error = 1; // max allowable translation error to pass [m]
	double max_yaw_error = 0.05; // max allowable yaw error [rad]



	// start timer.
	int start = clock();

	//set up parameters here
	double delta_t = 0.1; // time elapsed between measurements [sec]
	double sensor_range = 50; // sensor range [m]

	/*
	 * sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 */
	double sigma_pos[3] = { 0.3, 0.3, 0.01 }; // gps measurement uncertainty [x [m], y [m], theta [rad]]
	double sigma_landmark[2] = { 0.3, 0.3 }; // landmark measurement uncertainty [x [m], y [m]]

	// noise generation
	default_random_engine gen;
	normal_distribution<double> n_x_init(0, sigma_pos[0]);
	normal_distribution<double> n_y_init(0, sigma_pos[1]);
	normal_distribution<double> n_theta_init(0, sigma_pos[2]);
	normal_distribution<double> n_obs_x(0, sigma_landmark[0]);
	normal_distribution<double> n_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta, n_range, n_heading;
	// read map data
	TargetMap map;

	if (!read_map_data("data/map_data.txt", map)) {
		cout << "error: could not open map file" << endl;
		return;
	}
	for (int i = 0; i < map.landmark_list.size(); i++) {
		auto sprite = Sprite::create("dot.png");
		sprite->setPosition(map.landmark_list.at(i).x_f, map.landmark_list.at(i).y_f);
		this->addChild(sprite);
	}
	//
	//// read position data
	//vector<control_s> position_meas;
	//if (!read_control_data("data/control_data.txt", position_meas)) {
	//	cout << "error: could not open position/control measurement file" << endl;
	//	return;
	//}

	//// read ground truth data
	//vector<ground_truth> gt;
	//if (!read_gt_data("data/gt_data.txt", gt)) {
	//	cout << "error: could not open ground truth data file" << endl;
	//	return;
	//}

	//// run particle filter!
	//int num_time_steps = position_meas.size();
	//ParticleFilter pf;
	//double total_error[3] = { 0,0,0 };
	//double cum_mean_error[3] = { 0,0,0 };

	//for (int i = 0; i < num_time_steps; ++i) {
	//	cout << "time step: " << i << endl;
	//	// read in landmark observations for current time step.
	//	ostringstream file;
	//	file << "data/observation/observations_" << setfill('0') << setw(6) << i + 1 << ".txt";
	//	vector<LandmarkObs> observations;
	//	if (!read_landmark_data(file.str(), observations)) {
	//		cout << "error: could not open observation file " << i + 1 << endl;
	//		return;
	//	}

	//	// initialize particle filter if this is the first time step.
	//	if (!pf.initialized()) {
	//		n_x = n_x_init(gen);
	//		n_y = n_y_init(gen);
	//		n_theta = n_theta_init(gen);
	//		pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);
	//	}
	//	else {
	//		// predict the vehicle's next state (noiseless).
	//		pf.prediction(delta_t, sigma_pos, position_meas[i - 1].velocity, position_meas[i - 1].yawrate);
	//	}
	//	// simulate the addition of noise to noiseless observation data.
	//	vector<LandmarkObs> noisy_observations;
	//	LandmarkObs obs;
	//	for (int j = 0; j < observations.size(); ++j) {
	//		n_x = n_obs_x(gen);
	//		n_y = n_obs_y(gen);
	//		obs = observations[j];
	//		obs.x = obs.x + n_x;
	//		obs.y = obs.y + n_y;
	//		noisy_observations.push_back(obs);
	//	}

	//	// update the weights and resample
	//	pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
	//	pf.resample();

	//	// calculate and output the average weighted error of the particle filter over all time steps so far.
	//	vector<Particle> particles = pf.particles;
	//	int num_particles = particles.size();
	//	double highest_weight = 0.0;
	//	Particle best_particle;
	//	for (int i = 0; i < num_particles; ++i) {
	//		if (particles[i].weight > highest_weight) {
	//			highest_weight = particles[i].weight;
	//			best_particle = particles[i];
	//		}
	//	}
	//	double* avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y, best_particle.theta);

	//	for (int j = 0; j < 3; ++j) {
	//		total_error[j] += avg_error[j];
	//		cum_mean_error[j] = total_error[j] / (double)(i + 1);
	//	}

	//	// print the cumulative weighted error
	//	cout << "cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;

	//	// if the error is too high, say so and then exit.
	//	if (i >= time_steps_before_lock_required) {
	//		if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error || cum_mean_error[2] > max_yaw_error) {
	//			if (cum_mean_error[0] > max_translation_error) {
	//				cout << "your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, " << max_translation_error << endl;
	//			}
	//			else if (cum_mean_error[1] > max_translation_error) {
	//				cout << "your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, " << max_translation_error << endl;
	//			}
	//			else {
	//				cout << "your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, " << max_yaw_error << endl;
	//			}
	//			return;
	//		}
	//	}
	//}

	//// output the runtime for the filter.
	//int stop = clock();
	//double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	//cout << "runtime (sec): " << runtime << endl;

	//// print success if accuracy and runtime are sufficient (and this isn't just the starter code).
	//if (runtime < max_runtime && pf.initialized()) {
	//	cout << "success! your particle filter passed!" << endl;
	//}
	//else if (!pf.initialized()) {
	//	cout << "this is the starter code. you haven't initialized your filter." << endl;
	//}
	//else {
	//	cout << "your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
	//	return;
	//}

	//return;
}

void HelloWorld::menuCloseCallback(Ref* pSender)
{
	//Close the cocos2d-x game scene and quit the application
	Director::getInstance()->end();

	/*To navigate back to native iOS screen(if present) without quitting the application  ,do not use Director::getInstance()->end() as given above,instead trigger a custom event created in RootViewController.mm as below*/

	//EventCustom customEndEvent("game_scene_close_event");
	//_eventDispatcher->dispatchEvent(&customEndEvent);


}
