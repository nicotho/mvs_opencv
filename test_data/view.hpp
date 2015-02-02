#ifndef VIEW_H
#define VIEW_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Imagine/Images.h>
#include <Imagine/Geometry.h>
#include <boost/filesystem.hpp>


class View
{
public:
	typedef Imagine::Image<Imagine::Color> Image;
	typedef Imagine::Camera<double> Camera;
	typedef Camera::Vector3 Vector3;

protected:
	Camera _camera;
	Image _image;
	std::string _image_path;
	std::string _camera_path;
	int _width, _height, _xmin, _ymin, _mipmap;
	double _znear, _zfar;

public:
	int width() const { return _width>>_mipmap; }
	int height() const { return _height>>_mipmap; }
	const std::string& imagePath() const { return _image_path; }
	const std::string& cameraPath() const { return _camera_path; }
	const Image& image() const { return _image; }
	Image& image() { return _image; }
	const Camera& camera() const { return _camera; }
	Camera& camera() { return _camera; }
	double znear() const { return _znear; }
	double zfar() const { return _zfar; }	


	bool isCameraLoaded() const { return (_camera == Camera()); }


	bool loadCamera()
	{
		if (boost::filesystem::extension( boost::filesystem::path(_camera_path) ) == ".bin")
		{
			if (!Imagine::loadBinary(_camera, _camera_path)) return false;
		}
		else
		{
			if (!Imagine::loadText(_camera, _camera_path)) return false;
		}

		if (_xmin>0 || _ymin>0) 
			_camera.translateImage(-double(_xmin), -double(_ymin));

		if (_mipmap > 0)
		{
			_camera.translateImage(-double(1<<(_mipmap-1))+.5, -double(1<<(_mipmap-1))+.5); // beware of reduce() offsetting
			_camera.scaleImage(1. / double(1<<_mipmap));
		}
		return true;
	}


	void unloadCamera()
	{
		_camera = Camera();
	}


	bool isImageLoaded() const { return !_image.empty(); }


	bool loadImage()
	{
		if (!Imagine::load(_image, _image_path)) return false;

		if (_xmin>0 || _ymin>0 || _image.width()>_width || _image.height()>_height)
			_image = _image.getSubImage(_xmin,_ymin,_width,_height);

		if (_camera.hasDistortion())
			_image = correctRadialDistortion(_image, _camera);

		if (_mipmap > 0)
			_image = Imagine::reduce(_image, 1<<_mipmap);

		return true;
	}


	void unloadImage()
	{
		_image = Image();
	}


	template <class Container>
	static bool loadViews(std::string filename, Container& views, int mipmap = 0, int split = 1)
	{
		std::cout << "Loading list of views from '" << filename << "': "; std::cout.flush();
		std::ifstream in(filename.c_str());
		if (!in.is_open())
		{
			std::cerr << "Cannot open file '" << filename << "' for reading" << std::endl;
			return false;
		}

		// Read image and camera paths
		std::string s;
		getline(in, s);
		boost::filesystem::path image_path(s);
		getline(in, s);
		boost::filesystem::path camera_path(s);

		// Read number of views
		getline(in, s);
		std::istringstream ss(s);
		int nviews = 0;
		if (!(ss >> nviews) || nviews<=0)
		{
			std::cerr << "Invalid number of views" << std::endl;
			return false;
		}

		// Read view info
		const int m=128; // Overlap between sub images (if split>1)
		for (int i=0; i<nviews; i++)
		{
			typename Container::value_type view;
			view._mipmap = mipmap;
			int width, height;
			in >> s >> width >> height;
			view._image_path = (image_path / s).string();
			in  >> s >> view._znear >> view._zfar;
			view._camera_path = (camera_path / s).string();
			int stepx=(width-m)/split,subw=stepx+m;
			int stepy=(height-m)/split,subh=stepy+m;
			for (int a=0;a<split;a++)
				for (int b=0;b<split;b++)
				{
					view._width = subw;
					view._height = subh;
					view._xmin = (a<split-1)?a*stepx:width-subw;
					view._ymin = (b<split-1)?b*stepy:height-subh;
					views.push_back(view);
				}
		}

		if (in.fail())
		{
			std::cerr << "Input error" << std::endl;
			return false;
		}

		std::cout << nviews << " x " << split << "^2 = " << nviews*split*split << " views" << std::endl;
		return true;
	}
};

#endif