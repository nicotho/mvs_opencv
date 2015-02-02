class Extract_Cloud : public Algorithm
{
public:



	struct Point
	{
		Vector3 pos, normal;
		float conf;
		vector<int> visib;
		RGB<byte> color;

		bool operator == (const Point& p) const { return (pos == p.pos); }
		bool operator < (const Point& p) const { return lexicographical_compare(pos.begin(), pos.end(), p.pos.begin(), p.pos.end()); }
	};

	struct DPoint
	{
		Vector3 pos;
		pair<int,int> visib;
		float score;
	};


	Extract_Cloud()
	{
	}


	~Extract_Cloud()
	{
		
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O

	bool loadViews(string filename, int mipmap = 0)
	{
		if (!View::loadViews(filename, views, mipmap)) return false;

		// Load cameras
		for (int i=0; i<views.size(); i++)
			if (!views[i].loadCamera()) return false;

		return true;
	}

	bool loadPairs(string filename) { return pairs.load(filename); }
protected:
	CameraPairs pairs;
	vector<OpenGLView> views;
	vector<Point> cloud;
	vector<DPoint> points;
	double box_threshold; // Bounding box threshold, in pixels

	FrameBuffer fbo;
	VertexShader score_vshader;
	FragmentShader score_fshader, depth_fshader;	