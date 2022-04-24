#include "objloader.h"

// From tutorial https://www.geeksforgeeks.org/how-to-create-an-unordered_map-of-pairs-in-c/
struct hash_pair {
	//template <class T1, class T2, class T3>
	size_t operator()(const Vector3f& vec) const
	{
		auto hash1 = std::hash<float>{}(vec[0]);
		auto hash2 = std::hash<float>{}(vec[1]);
		auto hash3 = std::hash<float>{}(vec[2]);
		return hash1 ^ hash2 ^ hash3;
	}
};

void loadOBJ(std::string& filename, std::vector<MFloatPoint>& shape_normals) 
{
	std::vector<Vector3f> positions;
	std::vector<Vector3f> normals;
	std::vector<Vector2f> texcoords;
	std::unordered_map<Vector3f, bool, hash_pair> usedNormals;
	float x, y, z = 0.f;
	float u, v = 0.f;

	std::string v1str = "";
	std::string v2str = "";
	std::string v3str = "";
	int v1, v2, v3;
	int v1n, v2n, v3n;
	int v1t, v2t, v3t;
	Vector3f vert1, vert2, vert3;
	Vector3f vert1n, vert2n, vert3n;
	Vector2f vert1t, vert2t, vert3t;

	std::ifstream stream(filename);
	std::string line;

	while (std::getline(stream, line)) {
		if (line.empty()) {
			continue;
		}
		std::string token;
		std::stringstream ss(line);
		ss >> token;
		if (!token.compare("v")) {
			ss >> x;
			ss >> y;
			ss >> z;
			Vector3f v = Vector3f(x, y, z);
			positions.push_back(v);
		}
		else if (!token.compare("vn")) {
			ss >> x;
			ss >> y;
			ss >> z;
			Vector3f n = Vector3f(x, y, z);
			normals.push_back(n);
		}
		else if (!token.compare("vt")) {
			ss >> u;
			ss >> v;
			Vector2f t = Vector2f(u, v);
			texcoords.push_back(t);
		}
		else if (!token.compare("f")) {
			ss >> v1str;
			ss >> v2str;
			ss >> v3str;

			int v1Slash = v1str.find("/", 0);
			int v1tStart = v1Slash + 1;
			int v1tSlash = v1str.find("/", v1tStart);
			int v1nStart = v1tSlash + 1;
			v1 = std::stoi(v1str.substr(0, v1Slash), nullptr);
			v1t = std::stoi(v1str.substr(v1tStart, v1tSlash - v1tStart), nullptr);
			v1n = std::stoi(v1str.substr(v1nStart), nullptr);

			int v2Slash = v2str.find("/", 0);
			int v2tStart = v2Slash + 1;
			int v2tSlash = v2str.find("/", v2tStart);
			int v2nStart = v2tSlash + 1;
			v2 = std::stoi(v2str.substr(0, v2Slash), nullptr);
			v2t = std::stoi(v2str.substr(v2tStart, v2tSlash - v2tStart), nullptr);
			v2n = std::stoi(v2str.substr(v2nStart), nullptr);

			int v3Slash = v3str.find("/", 0);
			int v3tStart = v3Slash + 1;
			int v3tSlash = v3str.find("/", v3tStart);
			int v3nStart = v3tSlash + 1;
			v3 = std::stoi(v3str.substr(0, v3Slash), nullptr);
			v3t = std::stoi(v3str.substr(v3tStart, v3tSlash - v3tStart), nullptr);
			v3n = std::stoi(v3str.substr(v3nStart), nullptr);

			if (v1 < 0) {
				vert1 = positions.at(positions.size() + v1);
			}
			else {
				vert1 = positions.at(v1 - 1);
			}
			if (v2 < 0) {
				vert2 = positions.at(positions.size() + v2);
			}
			else {
				vert2 = positions.at(v2 - 1);
			}
			if (v3 < 0) {
				vert3 = positions.at(positions.size() + v3);
			}
			else {
				vert3 = positions.at(v3 - 1);
			}
			if (v1t < 0) {
				vert1t = texcoords.at(texcoords.size() + v1t);
			}
			else {
				vert1t = texcoords.at(v1t - 1);
			}
			if (v2t < 0) {
				vert2t = texcoords.at(texcoords.size() + v2t);
			}
			else {
				vert2t = texcoords.at(v2t - 1);
			}
			if (v3t < 0) {
				vert3t = texcoords.at(texcoords.size() + v3t);
			}
			else {
				vert3t = texcoords.at(v3t - 1);
			}
			if (v1n < 0) {
				vert1n = normals.at(normals.size() + v1n);
			}
			else {
				vert1n = normals.at(v1n - 1);
			}
			if (v2n < 0) {
				vert2n = normals.at(normals.size() + v2n);
			}
			else {
				vert2n = normals.at(v2n - 1);
			}
			if (v3n < 0) {
				vert3n = normals.at(normals.size() + v3n);
			}
			else {
				vert3n = normals.at(v3n - 1);
			}

			// Get face normals
			Vector3f edge1 = vert1 - vert2;
			Vector3f edge2 = vert2 - vert3;
			Vector3f facenormal = edge1.cross(edge2);
			facenormal.normalize();

			// Insert into map
			if (usedNormals.find(facenormal) == usedNormals.end()) {
				usedNormals.insert(std::make_pair(facenormal, true));
				MFloatPoint fn = { facenormal[0], facenormal[1], facenormal[2] };
				shape_normals.push_back(fn);
			}		
			
		}
		else {
			// ignore line 
		}
	}
}