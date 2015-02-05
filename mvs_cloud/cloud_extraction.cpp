#include <iostream>
#include<string>



#include "mvs_cloud.hpp"



int main (int argc , char **argv)
{
	

	mvs_cloud cloud;
	cloud.compute_img_features();
	cloud.compute_cloud();
	std::cout<<cloud;
	




}





