
#include "URDFModel.h"
using namespace ofxRobotArm;
URDFModel::URDFModel()
{

}

URDFModel::~URDFModel()
{

}
void URDFModel::load(string filepath)
{
    ofxXmlSettings xml;
    ofLog(OF_LOG_NOTICE) << filepath << endl;
    if (xml.load(ofToDataPath(filepath)))
    {
        ofLog(OF_LOG_NOTICE) << "loaded" << endl;
        xml.pushTag("robot");
        int numLinks = xml.getNumTags("link");
        int numJoints = xml.getNumTags("joint");
        ofLog(OF_LOG_NOTICE) << numJoints << endl;
        pose.resize(numJoints);
        jointMin.resize(numJoints);
        jointMax.resize(numJoints);
        nodes.resize(numJoints);
        poseRadians.resize(numJoints);

        for (int i = 0; i < numJoints; i++)
        {
            xml.pushTag("joint", i);
            ofLog() << i << endl;
            string xyz = xml.getAttribute("origin", "xyz", "0.0 0.0 0.0", 0);
            string rot = xml.getAttribute("origin", "rpy", "0.0 0.0 0.0", 0);
            cout << xyz << endl;
            jointMin[i] = xml.getAttribute("limit", "lower", -TWO_PI, 0);
            jointMax[i] = xml.getAttribute("limit", "upper", TWO_PI, 0);
            string axis = xml.getAttribute("axis", "xyz", "0.0 0.0 0.0", 0);
            ofLog() << axis << endl;
            xml.popTag();
            Pose p;
            ofNode node = ofNode();
            vector<string> pos = ofSplitString(xyz, " ");
            p.position = ofVec3f(ofToFloat(pos[0]), ofToFloat(pos[1]), ofToFloat(pos[2])) * 1000;
            ofLog() << p.position << endl;
            node.setPosition(p.position);
            vector<string> ax = ofSplitString(axis, " ");
            p.axis = ofVec3f(ofToFloat(ax[0]), ofToFloat(ax[1]), ofToFloat(ax[2]));
            vector<string> ro = ofSplitString(rot, " ");
             ofLog() <<"ro "<<rot<< endl;
            p.rotation = 0;
            p.orientation.makeRotate(ofRadToDeg(ofToFloat(ro[0])), ofVec3f(1, 0, 0),
                                     ofRadToDeg(ofToFloat(ro[1])), ofVec3f(0, 1, 0),
                                     ofRadToDeg(ofToFloat(ro[2])), ofVec3f(0, 0, 1));
            
            if (i > 0)
            {
                p.offset = p.position - pose[i - 1].position;
                ofLog() <<"OFFSET "<< p.offset << endl;
                node.setParent(nodes[i - 1]);
            }
            pose[i] = p;
            nodes[i] = node;
        }

        for (int i = 0; i < numLinks; i++)
        {
            if(xml.pushTag("link", i))
            {
                ofxAssimpModelLoader loader;
                if(xml.pushTag("visual"))
                {
                    if(xml.pushTag("geometry"));
                    {
                        string path = ofToDataPath(xml.getAttribute("mesh", "filename", "", 0));
                        if (path != "")
                        {
                            ofLog(OF_LOG_NOTICE) << path << endl;
                            if(loader.loadModel(ofToDataPath(path))){
                                ofLog(OF_LOG_NOTICE) << "LOADED" << endl;
                                ofMesh m;
                                for(int i = 0 ; i < loader.getMeshCount(); i++){
                                    m.append(loader.getMesh(i));
                                     ofLog(OF_LOG_NOTICE) << "MESH "<< endl;
                                }
                                 ofLog(OF_LOG_NOTICE) <<m.getNumVertices()<<endl;
                                meshes.push_back(m); 
                            }else{
                                ofLog(OF_LOG_NOTICE) << "NOT LOADED!"<< endl;
                            }
                        }
                        xml.popTag();
                    }
                    xml.popTag(); 
                }
                xml.popTag();
            }
        }
    }
}

bool URDFModel::setup(string path, bool forceFixedBase, bool mergeFixedJoints, bool printDebug, bool parseSensors){
    mFlag = flag;
    parser.setSourceFile(path);
    ofFile file;
    file.open(ofToDataPath(path), ofFile::ReadOnly, false);
    ofBuffer buff = file.readToBuffer();
    std::string xml = buff.getText();
    bool result;
    if (xml.length())
	{
			result = parser.loadUrdf(xml.c_str(), forceFixedBase, parseSensors);
			if (parser.getModel().m_rootLinks.size())
			{
				if (mergeFixedJoints)
				{
					parser.mergeFixedLinks(parser.getModel(), parser.getModel().m_rootLinks[0], forceFixedBase, 0);
					parser.getModel().m_links.clear();
					parser.getModel().m_joints.clear();
					parser.recreateModel(parser.getModel(), parser.getModel().m_rootLinks[0]);
				}
				if (printDebug)
				{
					parser.printTree(parser.getModel().m_rootLinks[0], 0);
				}
			}
    }

    return result;
}
