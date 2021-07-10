#include "URDFParser.h"
#include "tinyxml2.h"
#include "urdfStringSplit.h"
#include "urdfLexicalCast.h"
#include "URDFFindMeshFile.h"

using namespace tinyxml2;

UrdfParser::UrdfParser()
	: m_parseSDF(false),
	  m_activeSdfModel(-1),
	  m_urdfScaling(1)
{
	m_urdf2Model.m_sourceFile = "IN_MEMORY_STRING"; // if loadUrdf() called later, source file name will be replaced with real
}

UrdfParser::~UrdfParser()
{
	for (int i = 0; i < m_tmpModels.size(); i++)
	{
		delete m_tmpModels[i];
	}
}

static bool parseVector4(ofVec4f &vec4, const std::string &vector_str)
{
	vec4.set(0, 0, 0, 0);
	std::vector<std::string> pieces;
	std::vector<double> rgba;
	std::vector<std::string> strArray;
	urdfIsAnyOf(" ", strArray);
	urdfStringSplit(pieces, vector_str, strArray);
	for (int i = 0; i < pieces.size(); ++i)
	{
		if (!pieces[i].empty())
		{
			rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
		}
	}
	if (rgba.size() != 4)
	{
		return false;
	}
	vec4.set(rgba[0], rgba[1], rgba[2], rgba[3]);
	return true;
}

static bool parseVector3(ofVec3f &vec3, const std::string &vector_str, bool lastThree = false)
{
	vec3.set(0, 0, 0);
	std::vector<std::string> pieces;
	std::vector<double> rgba;
	std::vector<std::string> strArray;
	urdfIsAnyOf(" ", strArray);
	urdfStringSplit(pieces, vector_str, strArray);
	for (int i = 0; i < pieces.size(); ++i)
	{
		if (!pieces[i].empty())
		{
			rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
		}
	}
	if (rgba.size() < 3)
	{
		ofLogWarning()<<("Couldn't parse vector3");
		return false;
	}
	if (lastThree)
	{
		vec3.set(rgba[rgba.size() - 3], rgba[rgba.size() - 2], rgba[rgba.size() - 1]);
	}
	else
	{
		vec3.set(rgba[0], rgba[1], rgba[2]);
	}
	return true;
}

// Parses user data from an xml element and stores it in a hashmap. User data is
// expected to reside in a <user-data> tag that is nested inside a <bullet> tag.
// Example:
// <bullet>
//   <user-data key="label">...</user-data>
// </bullet>
static void ParseUserData(const XMLElement *element, std::map<std::string, std::string> &user_data)
{
	// Parse any custom Bullet-specific info.
	for (const XMLElement *bullet_xml = element->FirstChildElement("bullet");
		 bullet_xml; bullet_xml = bullet_xml->NextSiblingElement("bullet"))
	{
		for (const XMLElement *user_data_xml = bullet_xml->FirstChildElement("user-data");
			 user_data_xml; user_data_xml = user_data_xml->NextSiblingElement("user-data"))
		{
			const char *key_attr = user_data_xml->Attribute("key");
			if (!key_attr)
			{
				ofLogError() << ("User data tag must have a key attribute.");
			}
			const char *text = user_data_xml->GetText();
			user_data.insert(std::pair<std::string, std::string>(key_attr, text ? text : ""));
		}
	}
}

bool UrdfParser::parseMaterial(UrdfMaterial &material, XMLElement *config)
{
	if (!config->Attribute("name"))
	{
		ofLogError() << ("Material must contain a name attribute");
		return false;
	}

	material.m_name = config->Attribute("name");

	// texture
	XMLElement *t = config->FirstChildElement("texture");
	if (t)
	{
		if (t->Attribute("filename"))
		{
			material.m_textureFilename = t->Attribute("filename");
		}
	}

	if (material.m_textureFilename.length() == 0)
	{
		//ofLogWarning()<<("material has no texture file name");
	}

	// color
	{
		XMLElement *c = config->FirstChildElement("color");
		if (c)
		{
			if (c->Attribute("rgba"))
			{
				if (!parseVector4(material.m_matColor.m_rgbaColor, c->Attribute("rgba")))
				{
					std::string msg = material.m_name + " has no rgba";
					ofLogWarning()<<(msg);
				}
			}
		}
	}

	{
		// specular (non-standard)
		XMLElement *s = config->FirstChildElement("specular");
		if (s)
		{
			if (s->Attribute("rgb"))
			{
				if (!parseVector3(material.m_matColor.m_specularColor, s->Attribute("rgb")))
				{
				}
			}
		}
	}
	return true;
}

bool UrdfParser::parseTransform(ofNode &tr, XMLElement *xml, bool parseSDF)
{
	tr.resetTransform();

	ofVec3f vec(0, 0, 0);
	if (parseSDF)
	{
		parseVector3(vec, std::string(xml->GetText()));
	}
	else
	{
		const char *xyz_str = xml->Attribute("xyz");
		if (xyz_str)
		{
			parseVector3(vec, std::string(xyz_str));
		}
	}
	tr.setPosition(vec * m_urdfScaling);

	if (parseSDF)
	{
		ofVec3f rpy;
		if (parseVector3(rpy, std::string(xml->GetText()), true))
		{
			double phi, the, psi;
			double roll = rpy[0];
			double pitch = rpy[1];
			double yaw = rpy[2];

			phi = roll / 2.0;
			the = pitch / 2.0;
			psi = yaw / 2.0;

			ofQuaternion orn(
				sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
				cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
				cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
				cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi));

			tr.setOrientation(orn);
		}
	}
	else
	{
		const char *rpy_str = xml->Attribute("rpy");
		if (rpy_str != NULL)
		{
			ofVec3f rpy;
			if (parseVector3(rpy, std::string(rpy_str)))
			{
				double phi, the, psi;
				double roll = rpy[0];
				double pitch = rpy[1];
				double yaw = rpy[2];

				phi = roll / 2.0;
				the = pitch / 2.0;
				psi = yaw / 2.0;

				ofQuaternion orn(
					sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
					cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
					cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
					cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi));

				tr.setOrientation(orn);
			}
		}
	}
	return true;
}

bool UrdfParser::parseInertia(UrdfInertia &inertia, XMLElement *config)
{
	inertia.m_linkLocalFrame.resetTransform();
	inertia.m_mass = 0.f;
	if (m_parseSDF)
	{
		XMLElement *pose = config->FirstChildElement("pose");
		if (pose)
		{
			parseTransform(inertia.m_linkLocalFrame, pose, m_parseSDF);
		}
	}

	// Origin
	XMLElement *o = config->FirstChildElement("origin");
	if (o)
	{
		if (!parseTransform(inertia.m_linkLocalFrame, o))
		{
			return false;
		}
	}

	XMLElement *mass_xml = config->FirstChildElement("mass");
	if (!mass_xml)
	{
		ofLogError() << ("Inertial element must have a mass element");
		return false;
	}
	if (m_parseSDF)
	{
		inertia.m_mass = urdfLexicalCast<double>(mass_xml->GetText());
	}
	else
	{
		if (!mass_xml->Attribute("value"))
		{
			ofLogError() << ("Inertial: mass element must have value attribute");
			return false;
		}

		inertia.m_mass = urdfLexicalCast<double>(mass_xml->Attribute("value"));
	}

	XMLElement *inertia_xml = config->FirstChildElement("inertia");
	if (!inertia_xml)
	{
		ofLogError() << ("Inertial element must have inertia element");
		return false;
	}
	if (m_parseSDF)
	{
		XMLElement *ixx = inertia_xml->FirstChildElement("ixx");
		XMLElement *ixy = inertia_xml->FirstChildElement("ixy");
		XMLElement *ixz = inertia_xml->FirstChildElement("ixz");
		XMLElement *iyy = inertia_xml->FirstChildElement("iyy");
		XMLElement *iyz = inertia_xml->FirstChildElement("iyz");
		XMLElement *izz = inertia_xml->FirstChildElement("izz");
		if (ixx && ixy && ixz && iyy && iyz && izz)
		{
			inertia.m_ixx = urdfLexicalCast<double>(ixx->GetText());
			inertia.m_ixy = urdfLexicalCast<double>(ixy->GetText());
			inertia.m_ixz = urdfLexicalCast<double>(ixz->GetText());
			inertia.m_iyy = urdfLexicalCast<double>(iyy->GetText());
			inertia.m_iyz = urdfLexicalCast<double>(iyz->GetText());
			inertia.m_izz = urdfLexicalCast<double>(izz->GetText());
		}
		else
		{
			if (ixx && iyy && izz)
			{
				inertia.m_ixx = urdfLexicalCast<double>(ixx->GetText());
				inertia.m_ixy = 0;
				inertia.m_ixz = 0;
				inertia.m_iyy = urdfLexicalCast<double>(iyy->GetText());
				inertia.m_iyz = 0;
				inertia.m_izz = urdfLexicalCast<double>(izz->GetText());
			}
			else
			{
				ofLogError() << ("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz child elements");
				return false;
			}
		}
	}
	else
	{
		if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
			  inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
			  inertia_xml->Attribute("izz")))
		{
			if ((inertia_xml->Attribute("ixx") && inertia_xml->Attribute("iyy") &&
				 inertia_xml->Attribute("izz")))
			{
				inertia.m_ixx = urdfLexicalCast<double>(inertia_xml->Attribute("ixx"));
				inertia.m_ixy = 0;
				inertia.m_ixz = 0;
				inertia.m_iyy = urdfLexicalCast<double>(inertia_xml->Attribute("iyy"));
				inertia.m_iyz = 0;
				inertia.m_izz = urdfLexicalCast<double>(inertia_xml->Attribute("izz"));
			}
			else
			{
				ofLogError() << ("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
				return false;
			}
		}
		else
		{
			inertia.m_ixx = urdfLexicalCast<double>(inertia_xml->Attribute("ixx"));
			inertia.m_ixy = urdfLexicalCast<double>(inertia_xml->Attribute("ixy"));
			inertia.m_ixz = urdfLexicalCast<double>(inertia_xml->Attribute("ixz"));
			inertia.m_iyy = urdfLexicalCast<double>(inertia_xml->Attribute("iyy"));
			inertia.m_iyz = urdfLexicalCast<double>(inertia_xml->Attribute("iyz"));
			inertia.m_izz = urdfLexicalCast<double>(inertia_xml->Attribute("izz"));
		}
	}
	return true;
}

bool UrdfParser::parseGeometry(UrdfGeometry &geom, XMLElement *g)
{
	//	btAssert(g);
	if (g == 0)
		return false;

	XMLElement *shape = g->FirstChildElement();
	if (!shape)
	{
		ofLogError() << ("Geometry tag contains no child element.");
		return false;
	}

	//const std::string type_name = shape->ValueTStr();
	const std::string type_name = shape->Value();
	if (type_name == "sphere")
	{
		geom.m_type = URDF_GEOM_SPHERE;
		if (m_parseSDF)
		{
			XMLElement *size = shape->FirstChildElement("radius");
			if (0 == size)
			{
				ofLogError() << ("sphere requires a radius child element");
				return false;
			}
			geom.m_sphereRadius = urdfLexicalCast<double>(size->GetText());
		}
		else
		{
			if (!shape->Attribute("radius"))
			{
				ofLogError() << ("Sphere shape must have a radius attribute");
				return false;
			}
			else
			{
				geom.m_sphereRadius = m_urdfScaling * urdfLexicalCast<double>(shape->Attribute("radius"));
			}
		}
	}
	else if (type_name == "box")
	{
		geom.m_type = URDF_GEOM_BOX;
		if (m_parseSDF)
		{
			XMLElement *size = shape->FirstChildElement("size");
			if (0 == size)
			{
				ofLogError() << ("box requires a size child element");
				return false;
			}
			parseVector3(geom.m_boxSize, size->GetText());
			geom.m_boxSize *= m_urdfScaling;
		}
		else
		{
			if (!shape->Attribute("size"))
			{
				ofLogError() << ("box requires a size attribute");
				return false;
			}
			else
			{
				parseVector3(geom.m_boxSize, shape->Attribute("size"));
				geom.m_boxSize *= m_urdfScaling;
			}
		}
	}
	else if (type_name == "cylinder")
	{
		geom.m_type = URDF_GEOM_CYLINDER;
		geom.m_hasFromTo = false;
		geom.m_capsuleRadius = 0.1;
		geom.m_capsuleHeight = 0.1;

		if (m_parseSDF)
		{
			if (XMLElement *scale = shape->FirstChildElement("radius"))
			{
				parseVector3(geom.m_meshScale, scale->GetText());
				geom.m_capsuleRadius = m_urdfScaling * urdfLexicalCast<double>(scale->GetText());
			}
			if (XMLElement *scale = shape->FirstChildElement("length"))
			{
				parseVector3(geom.m_meshScale, scale->GetText());
				geom.m_capsuleHeight = m_urdfScaling * urdfLexicalCast<double>(scale->GetText());
			}
		}
		else
		{
			if (!shape->Attribute("length") || !shape->Attribute("radius"))
			{
				ofLogError() << ("Cylinder shape must have both length and radius attributes");
				return false;
			}
			geom.m_capsuleRadius = m_urdfScaling * urdfLexicalCast<double>(shape->Attribute("radius"));
			geom.m_capsuleHeight = m_urdfScaling * urdfLexicalCast<double>(shape->Attribute("length"));
		}
	}
	else if (type_name == "capsule")
	{
		geom.m_type = URDF_GEOM_CAPSULE;
		geom.m_hasFromTo = false;
		if (m_parseSDF)
		{
			if (XMLElement *scale = shape->FirstChildElement("radius"))
			{
				parseVector3(geom.m_meshScale, scale->GetText());
				geom.m_capsuleRadius = m_urdfScaling * urdfLexicalCast<double>(scale->GetText());
			}
			if (XMLElement *scale = shape->FirstChildElement("length"))
			{
				parseVector3(geom.m_meshScale, scale->GetText());
				geom.m_capsuleHeight = m_urdfScaling * urdfLexicalCast<double>(scale->GetText());
			}
		}
		else
		{
			if (!shape->Attribute("length") || !shape->Attribute("radius"))
			{
				ofLogError() << ("Capsule shape must have both length and radius attributes");
				return false;
			}
			geom.m_capsuleRadius = m_urdfScaling * urdfLexicalCast<double>(shape->Attribute("radius"));
			geom.m_capsuleHeight = m_urdfScaling * urdfLexicalCast<double>(shape->Attribute("length"));
		}
	}
	else if ((type_name == "mesh") || (type_name == "cdf"))
	{
		if ((type_name == "cdf"))
		{
			geom.m_type = URDF_GEOM_CDF;
		}
		else
		{
			geom.m_type = URDF_GEOM_MESH;
		}
		geom.m_meshScale.set(1, 1, 1);
		std::string fn;

		if (m_parseSDF)
		{
			if (XMLElement *scale = shape->FirstChildElement("scale"))
			{
				parseVector3(geom.m_meshScale, scale->GetText());
			}
			if (XMLElement *filename = shape->FirstChildElement("uri"))
			{
				fn = filename->GetText();
			}
		}
		else
		{
			// URDF
			if (shape->Attribute("filename"))
			{
				fn = shape->Attribute("filename");
			}
			if (shape->Attribute("scale"))
			{
				if (!parseVector3(geom.m_meshScale, shape->Attribute("scale")))
				{
					ofLogWarning()<<("Scale should be a vector3, not single scalar. Workaround activated.\n");
					std::string scalar_str = shape->Attribute("scale");
					double scaleFactor = urdfLexicalCast<double>(scalar_str.c_str());
					if (scaleFactor)
					{
						geom.m_meshScale.set(scaleFactor, scaleFactor, scaleFactor);
					}
				}
			}
		}

		geom.m_meshScale *= m_urdfScaling;

		if (fn.empty())
		{
			ofLogError() << ("Mesh filename is empty");
			return false;
		}

		geom.m_meshFileName = fn;
		bool success = UrdfFindMeshFile(m_urdf2Model.m_sourceFile, fn, sourceFileLocation(shape),
										&geom.m_meshFileName, &geom.m_meshFileType);
		if (!success)
		{
			// warning already printed
			return false;
		}
	}
	else
	{
		if (type_name == "plane")
		{
			geom.m_type = URDF_GEOM_PLANE;
			if (this->m_parseSDF)
			{
				XMLElement *n = shape->FirstChildElement("normal");
				XMLElement *s = shape->FirstChildElement("size");

				if ((0 == n) || (0 == s))
				{
					ofLogError() << ("Plane shape must have both normal and size attributes");
					return false;
				}

				parseVector3(geom.m_planeNormal, n->GetText());
			}
			else
			{
				if (!shape->Attribute("normal"))
				{
					ofLogError() << ("plane requires a normal attribute");
					return false;
				}
				else
				{
					parseVector3(geom.m_planeNormal, shape->Attribute("normal"));
				}
			}
		}
		else
		{
			ofLogError() << ("Unknown geometry type:");
			ofLogError() << (type_name);
			return false;
		}
	}

	return true;
}

bool UrdfParser::parseCollision(UrdfCollision &collision, XMLElement *config)
{
	collision.m_linkLocalFrame.resetTransform();

	if (m_parseSDF)
	{
		XMLElement *pose = config->FirstChildElement("pose");
		if (pose)
		{
			parseTransform(collision.m_linkLocalFrame, pose, m_parseSDF);
		}
	}

	// Origin
	XMLElement *o = config->FirstChildElement("origin");
	if (o)
	{
		if (!parseTransform(collision.m_linkLocalFrame, o))
			return false;
	}
	// Geometry
	XMLElement *geom = config->FirstChildElement("geometry");
	if (!parseGeometry(collision.m_geometry, geom))
	{
		return false;
	}

	{
		const char *group_char = config->Attribute("group");
		if (group_char)
		{
			collision.m_flags |= URDF_HAS_COLLISION_GROUP;
			collision.m_collisionGroup = urdfLexicalCast<int>(group_char);
		}
	}

	{
		const char *mask_char = config->Attribute("mask");
		if (mask_char)
		{
			collision.m_flags |= URDF_HAS_COLLISION_MASK;
			collision.m_collisionMask = urdfLexicalCast<int>(mask_char);
		}
	}

	const char *name_char = config->Attribute("name");
	if (name_char)
		collision.m_name = name_char;

	const char *concave_char = config->Attribute("concave");
	if (concave_char)
		collision.m_flags |= URDF_FORCE_CONCAVE_TRIMESH;

	return true;
}

bool UrdfParser::parseVisual(UrdfModel &model, UrdfVisual &visual, XMLElement *config)
{
	visual.m_linkLocalFrame.resetTransform();
	if (m_parseSDF)
	{
		XMLElement *pose = config->FirstChildElement("pose");
		if (pose)
		{
			parseTransform(visual.m_linkLocalFrame, pose, m_parseSDF);
		}
	}

	// Origin
	XMLElement *o = config->FirstChildElement("origin");
	if (o)
	{
		if (!parseTransform(visual.m_linkLocalFrame, o))
			return false;
	}
	// Geometry
	XMLElement *geom = config->FirstChildElement("geometry");
	if (!parseGeometry(visual.m_geometry, geom))
	{
		return false;
	}

	const char *name_char = config->Attribute("name");
	if (name_char)
		visual.m_name = name_char;

	visual.m_geometry.m_hasLocalMaterial = false;

	// Material
	XMLElement *mat = config->FirstChildElement("material");
	//todo(erwincoumans) skip materials in SDF for now (due to complexity)
	if (mat)
	{
		if (m_parseSDF)
		{
			UrdfMaterial *matPtr = new UrdfMaterial;
			matPtr->m_name = "mat";
			if (name_char)
				matPtr->m_name = name_char;

			UrdfMaterial *oldMatPtrPtr = model.m_materials[matPtr->m_name];
			if (oldMatPtrPtr)
			{
				UrdfMaterial *oldMatPtr = oldMatPtrPtr;
				model.m_materials.erase(matPtr->m_name);
				if (oldMatPtr)
					delete oldMatPtr;
			}
			model.m_materials.insert(std::pair<std::string, UrdfMaterial*>(matPtr->m_name, matPtr));
			{
				XMLElement *diffuse = mat->FirstChildElement("diffuse");
				if (diffuse)
				{
					std::string diffuseText = diffuse->GetText();
					ofVec4f rgba(1, 0, 0, 1);
					parseVector4(rgba, diffuseText);
					matPtr->m_matColor.m_rgbaColor = rgba;

					visual.m_materialName = matPtr->m_name;
					visual.m_geometry.m_hasLocalMaterial = true;
				}
			}
			{
				XMLElement *specular = mat->FirstChildElement("specular");
				if (specular)
				{
					std::string specularText = specular->GetText();
					ofVec3f rgba(1, 1, 1);
					parseVector3(rgba, specularText);
					matPtr->m_matColor.m_specularColor = rgba;
					visual.m_materialName = matPtr->m_name;
					visual.m_geometry.m_hasLocalMaterial = true;
				}
			}
		}
		else
		{
			// get material name
			if (!mat->Attribute("name"))
			{
				ofLogError() << ("Visual material must contain a name attribute");
				return false;
			}
			visual.m_materialName = mat->Attribute("name");

			// try to parse material element in place

			XMLElement *t = mat->FirstChildElement("texture");
			XMLElement *c = mat->FirstChildElement("color");
			XMLElement *s = mat->FirstChildElement("specular");
			if (t || c || s)
			{
				if (parseMaterial(visual.m_geometry.m_localMaterial, mat))
				{
					UrdfMaterial *matPtr = new UrdfMaterial(visual.m_geometry.m_localMaterial);

					UrdfMaterial *oldMatPtrPtr = model.m_materials[matPtr->m_name];
					if (oldMatPtrPtr)
					{
						UrdfMaterial *oldMatPtr = oldMatPtrPtr;
						model.m_materials.erase(matPtr->m_name);
						if (oldMatPtr)
							delete oldMatPtr;
					}
					model.m_materials.insert(std::pair<std::string, UrdfMaterial*>(matPtr->m_name, matPtr));
					visual.m_geometry.m_hasLocalMaterial = true;
				}
			}
		}
	}
	ParseUserData(config, visual.m_userData);

	return true;
}

bool UrdfParser::parseLink(UrdfModel &model, UrdfLink &link, XMLElement *config)
{
	const char *linkName = config->Attribute("name");
	if (!linkName)
	{
		ofLogError() << ("Link with no name");
		return false;
	}
	link.m_name = linkName;

	if (m_parseSDF)
	{
		XMLElement *pose = config->FirstChildElement("pose");
		if (0 == pose)
		{
			link.m_linkTransformInWorld.resetTransform();
		}
		else
		{
			parseTransform(link.m_linkTransformInWorld, pose, m_parseSDF);
		}
	}

	{
//optional 'audio_source' parameters
//modified version of SDF audio_source specification in //http://sdformat.org/spec?ver=1.6&elem=link
#if 0
		<audio_source>
          <uri>file://media/audio/cheer.mp3</uri>
          <pitch>2.0</pitch>
          <gain>1.0</gain>
          <loop>false</loop>
          <contact>
            <collision>collision</collision>
          </contact>
        </audio_source>
#endif
		XMLElement *ci = config->FirstChildElement("audio_source");
		if (ci)
		{
			link.m_audioSource.m_flags |= SDFAudioSource::SDFAudioSourceValid;

			const char *fn = ci->Attribute("filename");
			if (fn)
			{
				link.m_audioSource.m_uri = fn;
			}
			else
			{
				if (XMLElement *filename_xml = ci->FirstChildElement("uri"))
				{
					link.m_audioSource.m_uri = filename_xml->GetText();
				}
			}
			if (XMLElement *pitch_xml = ci->FirstChildElement("pitch"))
			{
				link.m_audioSource.m_pitch = urdfLexicalCast<double>(pitch_xml->GetText());
			}
			if (XMLElement *gain_xml = ci->FirstChildElement("gain"))
			{
				link.m_audioSource.m_gain = urdfLexicalCast<double>(gain_xml->GetText());
			}

			if (XMLElement *attack_rate_xml = ci->FirstChildElement("attack_rate"))
			{
				link.m_audioSource.m_attackRate = urdfLexicalCast<double>(attack_rate_xml->GetText());
			}

			if (XMLElement *decay_rate_xml = ci->FirstChildElement("decay_rate"))
			{
				link.m_audioSource.m_decayRate = urdfLexicalCast<double>(decay_rate_xml->GetText());
			}

			if (XMLElement *sustain_level_xml = ci->FirstChildElement("sustain_level"))
			{
				link.m_audioSource.m_sustainLevel = urdfLexicalCast<double>(sustain_level_xml->GetText());
			}

			if (XMLElement *release_rate_xml = ci->FirstChildElement("release_rate"))
			{
				link.m_audioSource.m_releaseRate = urdfLexicalCast<double>(release_rate_xml->GetText());
			}

			if (XMLElement *loop_xml = ci->FirstChildElement("loop"))
			{
				std::string looptxt = loop_xml->GetText();
				if (looptxt == "true")
				{
					link.m_audioSource.m_flags |= SDFAudioSource::SDFAudioSourceLooping;
				}
			}
			if (XMLElement *forceThreshold_xml = ci->FirstChildElement("collision_force_threshold"))
			{
				link.m_audioSource.m_collisionForceThreshold = urdfLexicalCast<double>(forceThreshold_xml->GetText());
			}
			//todo: see other audio_source children
			//pitch, gain, loop, contact
		}
	}
	{
		//optional 'contact' parameters
		XMLElement *ci = config->FirstChildElement("contact");
		if (ci)
		{
			XMLElement *damping_xml = ci->FirstChildElement("inertia_scaling");
			if (damping_xml)
			{
				if (m_parseSDF)
				{
					link.m_contactInfo.m_inertiaScaling = urdfLexicalCast<double>(damping_xml->GetText());
					link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_INERTIA_SCALING;
				}
				else
				{
					if (!damping_xml->Attribute("value"))
					{
						ofLogError() << ("Link/contact: damping element must have value attribute");
						return false;
					}

					link.m_contactInfo.m_inertiaScaling = urdfLexicalCast<double>(damping_xml->Attribute("value"));
					link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_INERTIA_SCALING;
				}
			}
			{
				XMLElement *friction_xml = ci->FirstChildElement("lateral_friction");
				if (friction_xml)
				{
					if (m_parseSDF)
					{
						link.m_contactInfo.m_lateralFriction = urdfLexicalCast<double>(friction_xml->GetText());
					}
					else
					{
						if (!friction_xml->Attribute("value"))
						{
							ofLogError() << ("Link/contact: lateral_friction element must have value attribute");
							return false;
						}

						link.m_contactInfo.m_lateralFriction = urdfLexicalCast<double>(friction_xml->Attribute("value"));
					}
				}
			}

			{
				XMLElement *rolling_xml = ci->FirstChildElement("rolling_friction");
				if (rolling_xml)
				{
					if (m_parseSDF)
					{
						link.m_contactInfo.m_rollingFriction = urdfLexicalCast<double>(rolling_xml->GetText());
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_ROLLING_FRICTION;
					}
					else
					{
						if (!rolling_xml->Attribute("value"))
						{
							ofLogError() << ("Link/contact: rolling friction element must have value attribute");
							return false;
						}

						link.m_contactInfo.m_rollingFriction = urdfLexicalCast<double>(rolling_xml->Attribute("value"));
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_ROLLING_FRICTION;
					}
				}
			}

			{
				XMLElement *restitution_xml = ci->FirstChildElement("restitution");
				if (restitution_xml)
				{
					if (m_parseSDF)
					{
						link.m_contactInfo.m_restitution = urdfLexicalCast<double>(restitution_xml->GetText());
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_RESTITUTION;
					}
					else
					{
						if (!restitution_xml->Attribute("value"))
						{
							ofLogError() << ("Link/contact: restitution element must have value attribute");
							return false;
						}

						link.m_contactInfo.m_restitution = urdfLexicalCast<double>(restitution_xml->Attribute("value"));
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_RESTITUTION;
					}
				}
			}

			{
				XMLElement *spinning_xml = ci->FirstChildElement("spinning_friction");
				if (spinning_xml)
				{
					if (m_parseSDF)
					{
						link.m_contactInfo.m_spinningFriction = urdfLexicalCast<double>(spinning_xml->GetText());
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_SPINNING_FRICTION;
					}
					else
					{
						if (!spinning_xml->Attribute("value"))
						{
							ofLogError() << ("Link/contact: spinning friction element must have value attribute");
							return false;
						}

						link.m_contactInfo.m_spinningFriction = urdfLexicalCast<double>(spinning_xml->Attribute("value"));
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_SPINNING_FRICTION;
					}
				}
			}
			{
				XMLElement *friction_anchor = ci->FirstChildElement("friction_anchor");
				if (friction_anchor)
				{
					link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_FRICTION_ANCHOR;
				}
			}
			{
				XMLElement *stiffness_xml = ci->FirstChildElement("stiffness");
				if (stiffness_xml)
				{
					if (m_parseSDF)
					{
						link.m_contactInfo.m_contactStiffness = urdfLexicalCast<double>(stiffness_xml->GetText());
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
					}
					else
					{
						if (!stiffness_xml->Attribute("value"))
						{
							ofLogError() << ("Link/contact: stiffness element must have value attribute");
							return false;
						}

						link.m_contactInfo.m_contactStiffness = urdfLexicalCast<double>(stiffness_xml->Attribute("value"));
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
					}
				}
			}
			{
				XMLElement *damping_xml = ci->FirstChildElement("damping");
				if (damping_xml)
				{
					if (m_parseSDF)
					{
						link.m_contactInfo.m_contactDamping = urdfLexicalCast<double>(damping_xml->GetText());
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
					}
					else
					{
						if (!damping_xml->Attribute("value"))
						{
							ofLogError() << ("Link/contact: damping element must have value attribute");
							return false;
						}

						link.m_contactInfo.m_contactDamping = urdfLexicalCast<double>(damping_xml->Attribute("value"));
						link.m_contactInfo.m_flags |= URDF_CONTACT_HAS_STIFFNESS_DAMPING;
					}
				}
			}
		}
	}

	// Inertial (optional)
	XMLElement *i = config->FirstChildElement("inertial");
	if (i)
	{
		if (!parseInertia(link.m_inertia, i))
		{
			ofLogError() << ("Could not parse inertial element for Link:");
			ofLogError() << (link.m_name);
			return false;
		}
	}
	else
	{
		if ((strlen(linkName) == 5) && (strncmp(linkName, "world", 5)) == 0)
		{
			link.m_inertia.m_mass = 0.f;
			link.m_inertia.m_linkLocalFrame.resetTransform();
			link.m_inertia.m_ixx = 0.f;
			link.m_inertia.m_iyy = 0.f;
			link.m_inertia.m_izz = 0.f;
		}
		else
		{
			ofLogWarning()<<("No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame");
			link.m_inertia.m_mass = 1.f;
			link.m_inertia.m_linkLocalFrame.resetTransform();
			link.m_inertia.m_ixx = 1.f;
			link.m_inertia.m_iyy = 1.f;
			link.m_inertia.m_izz = 1.f;
			ofLogWarning()<<(link.m_name);
		}
	}

	// Multiple Visuals (optional)
	for (XMLElement *vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
	{
		UrdfVisual visual;
		visual.m_sourceFileLocation = sourceFileLocation(vis_xml);

		if (parseVisual(model, visual, vis_xml))
		{
			link.m_visualArray.push_back(visual);
		}
		else
		{
			ofLogError() << ("Could not parse visual element for Link:");
			ofLogError() << (link.m_name);
			return false;
		}
	}

	// Multiple Collisions (optional)
	for (XMLElement *col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
	{
		UrdfCollision col;
		col.m_sourceFileLocation = sourceFileLocation(col_xml);

		if (parseCollision(col, col_xml))
		{
			link.m_collisionArray.push_back(col);
		}
		else
		{
			ofLogError() << ("Could not parse collision element for Link:");
			ofLogError() << (link.m_name);
			return false;
		}
	}
	ParseUserData(config, link.m_userData);
	return true;
}

bool UrdfParser::parseLameCoefficients(LameCoefficients &lameCoefficients, tinyxml2::XMLElement *config)
{
	const char *mu = config->Attribute("mu");
	const char *lambda = config->Attribute("lambda");
	const char *damping = config->Attribute("damping");
	if (!mu || !lambda)
	{
		ofLogError() << ("expected mu lambda for LameCoefficients.");
		return false;
	}
	lameCoefficients.mu = urdfLexicalCast<double>(mu);
	lameCoefficients.lambda = urdfLexicalCast<double>(lambda);
	if (damping)
		lameCoefficients.damping = urdfLexicalCast<double>(damping);
	else
		lameCoefficients.damping = 0;
	return true;
}

bool UrdfParser::parseDeformable(UrdfModel &model, tinyxml2::XMLElement *config)
{
	UrdfDeformable &deformable = model.m_deformable;
	const char *deformableName = config->Attribute("name");
	if (!deformableName)
	{
		ofLogError() << ("Deformable with no name");
		return false;
	}
	deformable.m_name = deformableName;

	XMLElement *i = config->FirstChildElement("inertial");
	if (!i)
	{
		ofLogError() << ("expected an inertial element");
		return false;
	}
	UrdfInertia inertia;
	if (!parseInertia(inertia, i))
	{
		ofLogError() << ("Could not parse inertial element for deformable:");
		ofLogError() << (deformable.m_name);
		return false;
	}
	deformable.m_mass = inertia.m_mass;

	XMLElement *collisionMargin_xml = config->FirstChildElement("collision_margin");
	if (collisionMargin_xml)
	{
		if (!collisionMargin_xml->Attribute("value"))
		{
			ofLogError() << ("collision_margin element must have value attribute");
			return false;
		}
		deformable.m_collisionMargin = urdfLexicalCast<double>(collisionMargin_xml->Attribute("value"));
	}

	XMLElement *friction_xml = config->FirstChildElement("friction");
	if (friction_xml)
	{
		if (!friction_xml->Attribute("value"))
		{
			ofLogError() << ("friction element must have value attribute");
			return false;
		}
		deformable.m_friction = urdfLexicalCast<double>(friction_xml->Attribute("value"));
	}

	XMLElement *repulsion_xml = config->FirstChildElement("repulsion_stiffness");
	if (repulsion_xml)
	{
		if (!repulsion_xml->Attribute("value"))
		{
			ofLogError() << ("repulsion_stiffness element must have value attribute");
			return false;
		}
		deformable.m_repulsionStiffness = urdfLexicalCast<double>(repulsion_xml->Attribute("value"));
	}

	XMLElement *grav_xml = config->FirstChildElement("gravity_factor");
	if (grav_xml)
	{
		if (!grav_xml->Attribute("value"))
		{
			ofLogError() << ("gravity_factor element must have value attribute");
			return false;
		}
		deformable.m_gravFactor = urdfLexicalCast<double>(grav_xml->Attribute("value"));
	}

	XMLElement *cache_barycenter = config->FirstChildElement("cache_barycenter");
	if (cache_barycenter)
	{
		deformable.m_cache_barycenter = true;
	}

	XMLElement *spring_xml = config->FirstChildElement("spring");
	if (spring_xml)
	{
		if (!spring_xml->Attribute("elastic_stiffness") || !spring_xml->Attribute("damping_stiffness"))
		{
			ofLogError() << ("spring element expect elastic and damping stiffness");
			return false;
		}

		deformable.m_springCoefficients.elastic_stiffness = urdfLexicalCast<double>(spring_xml->Attribute("elastic_stiffness"));
		deformable.m_springCoefficients.damping_stiffness = urdfLexicalCast<double>(spring_xml->Attribute("damping_stiffness"));

		if (spring_xml->Attribute("bending_stiffness"))
		{
			deformable.m_springCoefficients.bending_stiffness = urdfLexicalCast<double>(spring_xml->Attribute("bending_stiffness"));

			if (spring_xml->Attribute("bending_stride"))
				deformable.m_springCoefficients.bending_stride = urdfLexicalCast<int>(spring_xml->Attribute("bending_stride"));
		}
	}

	XMLElement *corotated_xml = config->FirstChildElement("corotated");
	if (corotated_xml)
	{
		if (!parseLameCoefficients(deformable.m_corotatedCoefficients, corotated_xml))
		{
			return false;
		}
	}

	XMLElement *neohookean_xml = config->FirstChildElement("neohookean");
	if (neohookean_xml)
	{
		if (!parseLameCoefficients(deformable.m_neohookeanCoefficients, neohookean_xml))
		{
			return false;
		}
	}

	XMLElement *vis_xml = config->FirstChildElement("visual");
	if (!vis_xml)
	{
		ofLogError() << ("expected an visual element");
		return false;
	}
	if (!vis_xml->Attribute("filename"))
	{
		ofLogError() << ("expected a filename for visual geometry");
		return false;
	}
	std::string fn = vis_xml->Attribute("filename");
	deformable.m_visualFileName = fn;

	int out_type(0);
	bool success = UrdfFindMeshFile(model.m_sourceFile, fn, sourceFileLocation(vis_xml),
									&deformable.m_visualFileName, &out_type);

	if (!success)
	{
		// warning already printed
		return false;
	}

	XMLElement *col_xml = config->FirstChildElement("collision");
	if (col_xml)
	{
		if (!col_xml->Attribute("filename"))
		{
			ofLogError() << ("expected a filename for collision geoemtry");
			return false;
		}
		fn = vis_xml->Attribute("filename");
		success = UrdfFindMeshFile(model.m_sourceFile, fn, sourceFileLocation(vis_xml),
								   &deformable.m_simFileName, &out_type);

		if (!success)
		{
			// warning already printed
			return false;
		}
	}

	ParseUserData(config, deformable.m_userData);
	return true;
}

bool UrdfParser::parseJointLimits(UrdfJoint &joint, XMLElement *config)
{
	joint.m_lowerLimit = 0.f;
	joint.m_upperLimit = -1.f;
	joint.m_effortLimit = 0.f;
	joint.m_velocityLimit = 0.f;
	joint.m_jointDamping = 0.f;
	joint.m_jointFriction = 0.f;

	if (m_parseSDF)
	{
		XMLElement *lower_xml = config->FirstChildElement("lower");
		if (lower_xml)
		{
			joint.m_lowerLimit = urdfLexicalCast<double>(lower_xml->GetText());
		}

		XMLElement *upper_xml = config->FirstChildElement("upper");
		if (upper_xml)
		{
			joint.m_upperLimit = urdfLexicalCast<double>(upper_xml->GetText());
		}

		XMLElement *effort_xml = config->FirstChildElement("effort");
		if (effort_xml)
		{
			joint.m_effortLimit = urdfLexicalCast<double>(effort_xml->GetText());
		}

		XMLElement *velocity_xml = config->FirstChildElement("velocity");
		if (velocity_xml)
		{
			joint.m_velocityLimit = urdfLexicalCast<double>(velocity_xml->GetText());
		}
	}
	else
	{
		const char *lower_str = config->Attribute("lower");
		if (lower_str)
		{
			joint.m_lowerLimit = urdfLexicalCast<double>(lower_str);
		}

		const char *upper_str = config->Attribute("upper");
		if (upper_str)
		{
			joint.m_upperLimit = urdfLexicalCast<double>(upper_str);
		}

		if (joint.m_type == URDFPrismaticJoint)
		{
			joint.m_lowerLimit *= m_urdfScaling;
			joint.m_upperLimit *= m_urdfScaling;
		}

		// Get joint effort limit
		const char *effort_str = config->Attribute("effort");
		if (effort_str)
		{
			joint.m_effortLimit = urdfLexicalCast<double>(effort_str);
		}

		// Get joint velocity limit
		const char *velocity_str = config->Attribute("velocity");
		if (velocity_str)
		{
			joint.m_velocityLimit = urdfLexicalCast<double>(velocity_str);
		}
	}

	return true;
}

bool UrdfParser::parseJointDynamics(UrdfJoint &joint, XMLElement *config)
{
	joint.m_jointDamping = 0;
	joint.m_jointFriction = 0;

	if (m_parseSDF)
	{
		XMLElement *damping_xml = config->FirstChildElement("damping");
		if (damping_xml)
		{
			joint.m_jointDamping = urdfLexicalCast<double>(damping_xml->GetText());
		}

		XMLElement *friction_xml = config->FirstChildElement("friction");
		if (friction_xml)
		{
			joint.m_jointFriction = urdfLexicalCast<double>(friction_xml->GetText());
		}

		if (damping_xml == NULL && friction_xml == NULL)
		{
			ofLogError() << ("joint dynamics element specified with no damping and no friction");
			return false;
		}
	}
	else
	{
		// Get joint damping
		const char *damping_str = config->Attribute("damping");
		if (damping_str)
		{
			joint.m_jointDamping = urdfLexicalCast<double>(damping_str);
		}

		// Get joint friction
		const char *friction_str = config->Attribute("friction");
		if (friction_str)
		{
			joint.m_jointFriction = urdfLexicalCast<double>(friction_str);
		}

		if (damping_str == NULL && friction_str == NULL)
		{
			ofLogError() << ("joint dynamics element specified with no damping and no friction");
			return false;
		}
	}

	return true;
}

bool UrdfParser::parseJoint(UrdfJoint &joint, XMLElement *config)
{
	// Get Joint Name
	const char *name = config->Attribute("name");
	if (!name)
	{
		ofLogError() << ("unnamed joint found");
		return false;
	}
	joint.m_name = name;
	joint.m_parentLinkToJointTransform.resetTransform();

	// Get transform from Parent Link to Joint Frame
	XMLElement *origin_xml = config->FirstChildElement("origin");
	if (origin_xml)
	{
		if (!parseTransform(joint.m_parentLinkToJointTransform, origin_xml))
		{
			ofLogError() << ("Malformed parent origin element for joint:");
			ofLogError() << (joint.m_name);
			return false;
		}
	}

	// Get Parent Link
	XMLElement *parent_xml = config->FirstChildElement("parent");
	if (parent_xml)
	{
		if (m_parseSDF)
		{
			joint.m_parentLinkName = std::string(parent_xml->GetText());
		}
		else
		{
			const char *pname = parent_xml->Attribute("link");
			if (!pname)
			{
				ofLogError() << ("no parent link name specified for Joint link. this might be the root?");
				ofLogError() << (joint.m_name);
				return false;
			}
			else
			{
				joint.m_parentLinkName = std::string(pname);
			}
		}
	}

	// Get Child Link
	XMLElement *child_xml = config->FirstChildElement("child");
	if (child_xml)
	{
		if (m_parseSDF)
		{
			joint.m_childLinkName = std::string(child_xml->GetText());
		}
		else
		{
			const char *pname = child_xml->Attribute("link");
			if (!pname)
			{
				ofLogError() << ("no child link name specified for Joint link [%s].");
				ofLogError() << (joint.m_name);
				return false;
			}
			else
			{
				joint.m_childLinkName = std::string(pname);
			}
		}
	}

	// Get Joint type
	const char *type_char = config->Attribute("type");
	if (!type_char)
	{
		ofLogError() << ("joint [%s] has no type, check to see if it's a reference.");
		ofLogError() << (joint.m_name);
		return false;
	}

	std::string type_str = type_char;
	if (type_str == "spherical")
		joint.m_type = URDFSphericalJoint;
	else if (type_str == "planar")
		joint.m_type = URDFPlanarJoint;
	else if (type_str == "floating")
		joint.m_type = URDFFloatingJoint;
	else if (type_str == "revolute")
		joint.m_type = URDFRevoluteJoint;
	else if (type_str == "continuous")
		joint.m_type = URDFContinuousJoint;
	else if (type_str == "prismatic")
		joint.m_type = URDFPrismaticJoint;
	else if (type_str == "fixed")
		joint.m_type = URDFFixedJoint;
	else
	{
		ofLogError() << ("Joint ");
		ofLogError() << (joint.m_name);
		ofLogError() << ("has unknown type:");
		ofLogError() << (type_str);
		return false;
	}

	if (m_parseSDF)
	{
		if (joint.m_type != URDFFloatingJoint && joint.m_type != URDFFixedJoint)
		{
			// axis
			XMLElement *axis_xml = config->FirstChildElement("axis");
			if (!axis_xml)
			{
				std::string msg("urdfdom: no axis element for Joint, defaulting to (1,0,0) axis");
				msg = msg + " " + joint.m_name + "\n";
				ofLogWarning()<<(msg);
				joint.m_localJointAxis.set(1, 0, 0);
			}
			else
			{
				XMLElement *xyz_xml = axis_xml->FirstChildElement("xyz");
				if (xyz_xml)
				{
					if (!parseVector3(joint.m_localJointAxis, std::string(xyz_xml->GetText())))
					{
						ofLogError() << ("Malformed axis element:");
						ofLogError() << (joint.m_name);
						ofLogError() << (" for joint:");
						ofLogError() << (xyz_xml->GetText());
						return false;
					}
				}

				XMLElement *limit_xml = axis_xml->FirstChildElement("limit");
				if (limit_xml)
				{
					if (joint.m_type != URDFContinuousJoint)
					{
						if (!parseJointLimits(joint, limit_xml))
						{
							ofLogError() << ("Could not parse limit element for joint:");
							ofLogError() << (joint.m_name);
							return false;
						}
					}
				}
				else if (joint.m_type == URDFRevoluteJoint)
				{
					ofLogError() << ("Joint is of type REVOLUTE but it does not specify limits");
					ofLogError() << (joint.m_name);
					return false;
				}
				else if (joint.m_type == URDFPrismaticJoint)
				{
					ofLogError() << ("Joint is of type PRISMATIC without limits");
					ofLogError() << (joint.m_name);
					return false;
				}

				XMLElement *prop_xml = axis_xml->FirstChildElement("dynamics");
				if (prop_xml)
				{
					if (!parseJointDynamics(joint, prop_xml))
					{
						ofLogError() << ("Could not parse dynamics element for joint:");
						ofLogError() << (joint.m_name);
						return false;
					}
				}
			}
		}
	}
	else
	{
		// Get Joint Axis
		if (joint.m_type != URDFFloatingJoint && joint.m_type != URDFFixedJoint)
		{
			// axis
			XMLElement *axis_xml = config->FirstChildElement("axis");
			if (!axis_xml)
			{
				std::string msg("urdfdom: no axis element for Joint, defaulting to (1,0,0) axis");
				msg = msg + " " + joint.m_name + "\n";
				ofLogWarning()<<(msg);
				joint.m_localJointAxis.set(1, 0, 0);
			}
			else
			{
				if (axis_xml->Attribute("xyz"))
				{
					if (!parseVector3(joint.m_localJointAxis, axis_xml->Attribute("xyz")))
					{
						ofLogError() << ("Malformed axis element:");
						ofLogError() << (joint.m_name);
						ofLogError() << (" for joint:");
						ofLogError() << (axis_xml->Attribute("xyz"));
						return false;
					}
				}
			}
		}

		// Get limit
		XMLElement *limit_xml = config->FirstChildElement("limit");
		if (limit_xml)
		{
			if (!parseJointLimits(joint, limit_xml))
			{
				ofLogError() << ("Could not parse limit element for joint:");
				ofLogError() << (joint.m_name);
				return false;
			}
		}
		else if (joint.m_type == URDFRevoluteJoint)
		{
			ofLogError() << ("Joint is of type REVOLUTE but it does not specify limits");
			ofLogError() << (joint.m_name);
			return false;
		}
		else if (joint.m_type == URDFPrismaticJoint)
		{
			ofLogError() << ("Joint is of type PRISMATIC without limits");
			ofLogError() << (joint.m_name);
			return false;
		}

		joint.m_jointDamping = 0;
		joint.m_jointFriction = 0;

		// Get Dynamics
		XMLElement *prop_xml = config->FirstChildElement("dynamics");
		if (prop_xml)
		{
			// Get joint damping
			const char *damping_str = prop_xml->Attribute("damping");
			if (damping_str)
			{
				joint.m_jointDamping = urdfLexicalCast<double>(damping_str);
			}

			// Get joint friction
			const char *friction_str = prop_xml->Attribute("friction");
			if (friction_str)
			{
				joint.m_jointFriction = urdfLexicalCast<double>(friction_str);
			}

			if (damping_str == NULL && friction_str == NULL)
			{
				ofLogError() << ("joint dynamics element specified with no damping and no friction");
				return false;
			}
		}
	}

	return true;
}

bool UrdfParser::parseSensor(UrdfModel &model, UrdfLink &link, UrdfJoint &joint, XMLElement *config)
{
	// Sensors are mapped to Links with a Fixed Joints connecting to the parents.
	// They has no extent or mass so they will work with the existing
	// model without affecting the system.
	ofLogError() << ("Adding Sensor ");
	const char *sensorName = config->Attribute("name");
	if (!sensorName)
	{
		ofLogError() << ("Sensor with no name");
		return false;
	}

	ofLogError() << (sensorName);
	link.m_name = sensorName;
	link.m_linkTransformInWorld.resetTransform();
	link.m_inertia.m_mass = 0.f;
	link.m_inertia.m_linkLocalFrame.resetTransform();
	link.m_inertia.m_ixx = 0.f;
	link.m_inertia.m_iyy = 0.f;
	link.m_inertia.m_izz = 0.f;

	// Get Parent Link
	XMLElement *parent_xml = config->FirstChildElement("parent");
	if (parent_xml)
	{
		if (m_parseSDF)
		{
			joint.m_parentLinkName = std::string(parent_xml->GetText());
		}
		else
		{
			const char *pname = parent_xml->Attribute("link");
			if (!pname)
			{
				ofLogError() << ("no parent link name specified for sensor. this might be the root?");
				ofLogError() << (joint.m_name);
				return false;
			}
			else
			{
				joint.m_parentLinkName = std::string(pname);
			}
		}
	}

	joint.m_name = std::string(sensorName).append("_Joint");
	joint.m_childLinkName = sensorName;
	joint.m_type = URDFFixedJoint;
	joint.m_localJointAxis.set(0, 0, 0);

	// Get transform from Parent Link to Joint Frame
	XMLElement *origin_xml = config->FirstChildElement("origin");
	if (origin_xml)
	{
		if (!parseTransform(joint.m_parentLinkToJointTransform, origin_xml))
		{
			ofLogError() << ("Malformed origin element for sensor:");
			ofLogError() << (joint.m_name);
			return false;
		}
	}

	return true;
}

static void CalculatePrincipalAxisTransform(float *masses, const ofNode *transforms, ofMatrix3x3 *inertiasIn, ofNode &principal, ofVec3f &inertiaOut)
{
	// int n = 2;

	// float totalMass = 0;
	// ofVec3f center(0, 0, 0);
	// int k;

	// for (k = 0; k < n; k++)
	// {
	// 	center += transforms[k].getPosition() * masses[k];
	// 	totalMass += masses[k];
	// }

	// center /= totalMass;
	// principal.setPosition(center);

	// ofMatrix3x3 tensor(0, 0, 0, 0, 0, 0, 0, 0, 0);
	// for (k = 0; k < n; k++)
	// {

	// 	const ofNode &t = transforms[k];
	// 	ofVec3f o = t.getPosition() - center;

	// 	//compute inertia tensor in coordinate system of parent
	// 	ofQuaternion q = t.getGlobalOrientation();

	// 	float d = q.length2();
	// 	float s = (2.0) / d;
	// 	float xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
	// 	float wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
	// 	float xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
	// 	float yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;

	// 	ofMatrix3x3 mat1(float(1.0) - (yy + zz), xy - wz, xz + wy,
	// 		xy + wz, float(1.0) - (xx + zz), yz - wx,
	// 		xz - wy, yz + wx, float(1.0) - (xx + yy));
	// 	ofMatrix3x3 j = mat1;
	// 	j *= inertiasIn[k]]
	// 	j = mat1 * j;

	// 	//add inertia tensor
	// 	tensor += j;

	// 	//compute inertia tensor of pointmass at o
	// 	float o2 = o.lengthSquared();
	// 	// j[0].set(o2, 0, 0);
	// 	// j[1].set(0, o2, 0);
	// 	// j[2].set(0, 0, o2);
	// 	// j[0] += o * -o.x();
	// 	// j[1] += o * -o.y();
	// 	// j[2] += o * -o.z();

	// 	j.set(o2, 0, 0, 0, o2, 0, 0, 0, o2);
		

	// 	//add inertia tensor of pointmass
	// 	tensor[0] += masses[k] * j[0];
	// 	tensor[1] += masses[k] * j[1];
	// 	tensor[2] += masses[k] * j[2];
	// }

	// tensor.diagonalize(principal.getBasis(), float(0.00001), 20);
	// inertiaOut.setValue(tensor[0][0], tensor[1][1], tensor[2][2]);
}

bool UrdfParser::mergeFixedLinks(UrdfModel &model, UrdfLink *link, bool forceFixedBase, int level)
{
	for (int l = 0; l < level; l++)
	{
		printf("\t");
	}
    ofLog()<<"processing "<<link->m_name<<endl;

	for (int i = 0; i < link->m_childJoints.size();)
	{
		if (link->m_childJoints[i]->m_type == URDFFixedJoint)
		{
			UrdfLink *childLink = link->m_childLinks[i];
			UrdfJoint *childJoint = link->m_childJoints[i];
			for (int l = 0; l < level + 1; l++)
			{
				printf("\t");
			}
			//mergeChildLink
            ofLog()<<"merge "<<childLink->m_name<<" into "<<link->m_name<<endl;
			for (int c = 0; c < childLink->m_collisionArray.size(); c++)
			{
				UrdfCollision col = childLink->m_collisionArray[c];

				ofMatrix4x4 mat = childJoint->m_parentLinkToJointTransform.getGlobalTransformMatrix() * col.m_linkLocalFrame.getGlobalTransformMatrix();
				col.m_linkLocalFrame.setPosition(mat.getTranslation());
				col.m_linkLocalFrame.setOrientation(mat.getRotate());
				link->m_collisionArray.push_back(col);
			}

			for (int c = 0; c < childLink->m_visualArray.size(); c++)
			{
				UrdfVisual viz = childLink->m_visualArray[c];
				ofMatrix4x4 mat = childJoint->m_parentLinkToJointTransform.getGlobalTransformMatrix() * viz.m_linkLocalFrame.getGlobalTransformMatrix();
				viz.m_linkLocalFrame.setPosition(mat.getTranslation());
				viz.m_linkLocalFrame.setOrientation(mat.getRotate());
				link->m_visualArray.push_back(viz);
			}

			if (!link->m_inertia.m_hasLinkLocalFrame)
			{
				link->m_inertia.m_linkLocalFrame.resetTransform();
			}
			if (!childLink->m_inertia.m_hasLinkLocalFrame)
			{
				childLink->m_inertia.m_linkLocalFrame.resetTransform();
			}
			//for a 'forceFixedBase' don't merge
			bool isStaticBase = false;
			if (forceFixedBase && link->m_parentJoint == 0)
				isStaticBase = true;
			if (link->m_inertia.m_mass == 0 && link->m_parentJoint == 0)
				isStaticBase = true;

			//skip the mass and inertia merge for a fixed base link
			if (!isStaticBase)
			{
				float masses[2] = {(float)link->m_inertia.m_mass, (float)childLink->m_inertia.m_mass};
				ofNode node;
				ofMatrix4x4 mat = childJoint->m_parentLinkToJointTransform.getGlobalTransformMatrix() * childLink->m_inertia.m_linkLocalFrame.getGlobalTransformMatrix();
				node.setPosition(mat.getTranslation());
				node.setOrientation(mat.getRotate());
				ofNode transforms[2] = {link->m_inertia.m_linkLocalFrame, node};
				ofMatrix3x3 inertiaLink(
					link->m_inertia.m_ixx, link->m_inertia.m_ixy, link->m_inertia.m_ixz,
					link->m_inertia.m_ixy, link->m_inertia.m_iyy, link->m_inertia.m_iyz,
					link->m_inertia.m_ixz, link->m_inertia.m_iyz, link->m_inertia.m_izz);
				ofMatrix3x3 inertiaChild(
					childLink->m_inertia.m_ixx, childLink->m_inertia.m_ixy, childLink->m_inertia.m_ixz,
					childLink->m_inertia.m_ixy, childLink->m_inertia.m_iyy, childLink->m_inertia.m_iyz,
					childLink->m_inertia.m_ixz, childLink->m_inertia.m_iyz, childLink->m_inertia.m_izz);
				ofMatrix3x3 inertiasIn[2] = {inertiaLink, inertiaChild};
				ofVec3f inertiaOut;
				ofNode principal;
				CalculatePrincipalAxisTransform(masses, transforms, inertiasIn, principal, inertiaOut);
				link->m_inertia.m_hasLinkLocalFrame = true;
				//link->m_inertia.m_linkLocalFrame = principal;
				link->m_inertia.m_linkLocalFrame.setPosition(principal.getPosition());

				link->m_inertia.m_ixx = inertiaOut[0];
				link->m_inertia.m_ixy = 0;
				link->m_inertia.m_ixz = 0;
				link->m_inertia.m_iyy = inertiaOut[1];
				link->m_inertia.m_iyz = 0;
				link->m_inertia.m_izz = inertiaOut[2];
				link->m_inertia.m_mass += childLink->m_inertia.m_mass;
			}
			
			link->m_childJoints.erase(link->m_childJoints.begin()+i);
			link->m_childLinks.erase(link->m_childLinks.begin()+i);

			for (int g = 0; g < childLink->m_childJoints.size(); g++)
			{
				UrdfLink *grandChildLink = childLink->m_childLinks[g];
				UrdfJoint *grandChildJoint = childLink->m_childJoints[g];
				for (int l = 0; l < level + 2; l++)
				{
					printf("\t");
				}
		
                ofLog()<<"relink "<<grandChildLink->m_name<<" from "<<childLink->m_name<<" to "<<link->m_name<<endl;

				grandChildJoint->m_parentLinkName = link->m_name;
				ofMatrix4x4 mat = childJoint->m_parentLinkToJointTransform.getGlobalTransformMatrix() * grandChildJoint->m_parentLinkToJointTransform.getGlobalTransformMatrix();
				grandChildJoint->m_parentLinkToJointTransform.setPosition(mat.getTranslation());
				grandChildJoint->m_parentLinkToJointTransform.setOrientation(mat.getRotate());

				grandChildLink->m_parentLink = link;
				grandChildLink->m_parentJoint->m_parentLinkName = link->m_name;

				link->m_childJoints.push_back(grandChildJoint);
				link->m_childLinks.push_back(grandChildLink);
			}
			delete model.m_links[childLink->m_name];
			model.m_links.erase(childLink->m_name);
			delete model.m_joints[childLink->m_name];
			model.m_joints.erase(childJoint->m_name);
		}
		else
		{
			//keep this link and recurse
			mergeFixedLinks(model, link->m_childLinks[i], forceFixedBase, level + 1);
			i++;
		}
	}
	return true;
}

const std::string sJointNames[] = {
	"unused",
	"URDFRevoluteJoint",
	"URDFPrismaticJoint",
	"URDFContinuousJoint",
	"URDFFloatingJoint",
	"URDFPlanarJoint",
	"URDFFixedJoint",
	"URDFSphericalJoint",
};

bool UrdfParser::printTree(UrdfLink *link, int level)
{

    ofLog()<<link->m_name<<" mass= "<<link->m_inertia.m_mass;
	for (int i = 0; i < link->m_childJoints.size(); i++)
	{
		printTree(link->m_childLinks[i], level + 1);
	}
	return true;
}

bool UrdfParser::recreateModel(UrdfModel &model, UrdfLink *link)
{
	if (!link->m_parentJoint)
	{
		link->m_linkIndex = model.m_links.size();
		model.m_links.insert(std::pair<std::string, UrdfLink*>(link->m_name, link));
	}
	for (int i = 0; i < link->m_childJoints.size(); i++)
	{
		link->m_childLinks[i]->m_linkIndex = model.m_links.size();
		std::string childName = link->m_childLinks[i]->m_name;
		UrdfLink *childLink = link->m_childLinks[i];
		model.m_links.insert(std::pair<std::string, UrdfLink*>(childName, childLink));
		std::string jointName = link->m_childLinks[i]->m_parentJoint->m_name;
		UrdfJoint *joint = link->m_childLinks[i]->m_parentJoint;
		model.m_joints.insert(std::pair<std::string, UrdfJoint*>(jointName, joint));
	}
	for (int i = 0; i < link->m_childJoints.size(); i++)
	{
		recreateModel(model, link->m_childLinks[i]);
	}
	return true;
}
bool UrdfParser::initTreeAndRoot(UrdfModel &model)
{
	// every link has children links and joints, but no parents, so we create a
	// local convenience data structure for keeping child->parent relations
	std::map<std::string, std::string> parentLinkTree;

	// loop through all joints, for every link, assign children links and children joints
	for (auto jointPtr : model.m_joints)
	{
		if (jointPtr.second)
		{
			UrdfJoint *joint = jointPtr.second;
			std::string parent_link_name = joint->m_parentLinkName;
			std::string child_link_name = joint->m_childLinkName;
			if (parent_link_name.empty() || child_link_name.empty())
			{
				ofLogError() << ("parent link or child link is empty for joint");
				ofLogError() << (joint->m_name);
				return false;
			}

			UrdfLink *childLinkPtr = model.m_links.find(joint->m_childLinkName)->second;
			if (!childLinkPtr)
			{
				ofLogError() << ("Cannot find child link for joint ");
				ofLogError() << (joint->m_name);

				return false;
			}

			UrdfLink *parentLinkPtr = model.m_links.find(joint->m_parentLinkName)->second;
			if (!parentLinkPtr)
			{
				ofLogError() << ("Cannot find parent link for a joint");
				ofLogError() << (joint->m_name);
				return false;
			}
			childLinkPtr->m_parentLink = parentLinkPtr;

			childLinkPtr->m_parentJoint = joint;
			parentLinkPtr->m_childJoints.push_back(joint);
			parentLinkPtr->m_childLinks.push_back(childLinkPtr);
			parentLinkTree.insert(std::pair<std::string, std::string>(childLinkPtr->m_name, parentLinkPtr->m_name));
		}
	}

	//search for children that have no parent, those are 'root'
	int i = 0;
	for (auto link : model.m_links)
	{
		if (link.second)
		{
			link.second->m_linkIndex = i;

			if (!link.second->m_parentLink)
			{
				model.m_rootLinks.push_back(link.second);
			}
		}
		i++;
	}

	if (model.m_rootLinks.size() > 1)
	{
		std::string multipleRootMessage =
			"URDF file with multiple root links found:";

		for (int i = 0; i < model.m_rootLinks.size(); i++)
		{
			multipleRootMessage += " ";
			multipleRootMessage += model.m_rootLinks[i]->m_name;
		}
		ofLog(OF_LOG_NOTICE) << multipleRootMessage << endl;
	}

	if (model.m_rootLinks.size() == 0)
	{
		ofLog(OF_LOG_NOTICE) << "URDF without root link found" << endl;
		return false;
	}

	return true;
}

bool UrdfParser::loadUrdf(const char *urdfText, bool forceFixedBase, bool parseSensors)
{
	XMLDocument xml_doc;
	xml_doc.Parse(urdfText);
	if (xml_doc.Error())
	{
#ifdef G3_TINYXML2
		ofLogError() << "xml reading error" << endl;
#else
		ofLogError() << xml_doc.ErrorStr() << endl;
		xml_doc.ClearError();
#endif
		return false;
	}

	XMLElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		ofLogError() << "expected a robot element" << endl;
		return false;
	}

	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		ofLogError() << "Expected a name for robot" << endl;
		return false;
	}
	m_urdf2Model.m_name = name;

	ParseUserData(robot_xml, m_urdf2Model.m_userData);

	// Get all Material elements
	for (XMLElement *material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
	{
		UrdfMaterial *material = new UrdfMaterial;

		parseMaterial(*material, material_xml);

		UrdfMaterial *mat = m_urdf2Model.m_materials.find(material->m_name)->second;
		if (mat)
		{
			delete material;
			ofLogError() << "Duplicate material" << endl;
		}
		else
		{
			m_urdf2Model.m_materials.insert(std::pair<std::string, UrdfMaterial*>(material->m_name, material));
		}
	}

	//	char msg[1024];
	//	sprintf(msg,"Num materials=%d", m_model.m_materials.size());
	//	logger->printMessage(msg);

	XMLElement *deformable_xml = robot_xml->FirstChildElement("deformable");
	if (deformable_xml)
		return parseDeformable(m_urdf2Model, deformable_xml);

	for (XMLElement *link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		UrdfLink *link = new UrdfLink;

		if (parseLink(m_urdf2Model, *link, link_xml))
		{
			if (m_urdf2Model.m_links.find(link->m_name) != m_urdf2Model.m_links.end())
			{
				ofLogError() << "Link name is not unique, link names in the same model have to be unique" << endl;
				ofLogError() << link->m_name << endl;
				delete link;
				return false;
			}
			else
			{
				//copy model material into link material, if link has no local material
				for (int i = 0; i < link->m_visualArray.size(); i++)
				{
					UrdfVisual &vis = link->m_visualArray.at(i);
					if (!vis.m_geometry.m_hasLocalMaterial && vis.m_materialName.size() > 0)
					{
						UrdfMaterial **mat = &m_urdf2Model.m_materials.find(vis.m_materialName)->second;
						if (mat && *mat)
						{
							vis.m_geometry.m_localMaterial = **mat;
						}
						else
						{
							//ofLogError()<<("Cannot find material with name:");
							//ofLogError()<<(vis.m_materialName);
						}
					}
				}

				m_urdf2Model.m_links.insert(std::pair<std::string,UrdfLink*>(link->m_name, link));
			}
		}
		else
		{
			ofLogError() << "failed to parse link" << endl;
			delete link;
			return false;
		}
	}
	if (m_urdf2Model.m_links.size() == 0)
	{
		ofLogWarning()<<("No links found in URDF file");
		return false;
	}

	// Get all Joint elements
	for (XMLElement *joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
	{
		UrdfJoint *joint = new UrdfJoint;

		if (parseJoint(*joint, joint_xml))
		{
			if (m_urdf2Model.m_joints.find(joint->m_name) != m_urdf2Model.m_joints.end())
			{
				ofLogError() << ("joint '%s' is not unique.");
				ofLogError() << (joint->m_name);
				delete joint;
				return false;
			}
			else
			{
				m_urdf2Model.m_joints.insert( std::pair<std::string, UrdfJoint*>(joint->m_name, joint));
			}
		}
		else
		{
			ofLogError() << ("joint xml is not initialized correctly");
			delete joint;
			return false;
		}
	}

	if (parseSensors)
	{
		// Get all Sensor Elements.
		for (XMLElement *sensor_xml = robot_xml->FirstChildElement("sensor"); sensor_xml; sensor_xml = sensor_xml->NextSiblingElement("sensor"))
		{
			UrdfLink *sensor = new UrdfLink;
			UrdfJoint *sensor_joint = new UrdfJoint;

			if (parseSensor(m_urdf2Model, *sensor, *sensor_joint, sensor_xml))
			{
				if (m_urdf2Model.m_links.find(sensor->m_name) != m_urdf2Model.m_links.end())
				{
					ofLogError() << ("Sensor name is not unique, sensor and link names in the same model have to be unique");
					ofLogError() << (sensor->m_name);
					delete sensor;
					delete sensor_joint;
					return false;
				}
				else if (m_urdf2Model.m_joints.find(sensor_joint->m_name) != m_urdf2Model.m_joints.end())
				{
					ofLogError() << ("Sensor Joint name is not unique, joint names in the same model have to be unique");
					ofLogError() << (sensor_joint->m_name);
					delete sensor;
					delete sensor_joint;
					return false;
				}
				else
				{
					m_urdf2Model.m_links.insert( std::pair<std::string, UrdfLink*>(sensor->m_name, sensor));
					m_urdf2Model.m_joints.insert( std::pair<std::string, UrdfJoint*>(sensor_joint->m_name, sensor_joint));
				}
			}
			else
			{
				ofLogError() << ("failed to parse sensor");
				delete sensor;
				delete sensor_joint;
				return false;
			}
		}
	}

	if (m_urdf2Model.m_links.size() == 0)
	{
		ofLogWarning()<<("No links found in URDF file");
		return false;
	}

	bool ok(initTreeAndRoot(m_urdf2Model));
	if (!ok)
	{
		return false;
	}

	if (forceFixedBase)
	{
		for (int i = 0; i < m_urdf2Model.m_rootLinks.size(); i++)
		{
			UrdfLink *link(m_urdf2Model.m_rootLinks.at(i));
			link->m_inertia.m_mass = 0.0;
			link->m_inertia.m_ixx = 0.0;
			link->m_inertia.m_ixy = 0.0;
			link->m_inertia.m_ixz = 0.0;
			link->m_inertia.m_iyy = 0.0;
			link->m_inertia.m_iyz = 0.0;
			link->m_inertia.m_izz = 0.0;
		}
	}

	return true;
}

void UrdfParser::activateModel(int modelIndex)
{
	m_activeSdfModel = modelIndex;
}

bool UrdfParser::loadSDF(const char *sdfText)
{
	XMLDocument xml_doc;
	xml_doc.Parse(sdfText);
	if (xml_doc.Error())
	{
#ifdef G3_TINYXML2
		ofLogError() << ("xml reading error");
#else
		ofLogError() << (xml_doc.ErrorStr());
		xml_doc.ClearError();
#endif
		return false;
	}

	XMLElement *sdf_xml = xml_doc.FirstChildElement("sdf");
	if (!sdf_xml)
	{
		ofLogError() << ("expected an sdf element");
		return false;
	}

	//apparently, SDF doesn't require a "world" element, optional? URDF does.
	XMLElement *world_xml = sdf_xml->FirstChildElement("world");

	XMLElement *robot_xml = 0;

	if (!world_xml)
	{
		ofLogWarning()<<("expected a world element, continuing without it.");
		robot_xml = sdf_xml->FirstChildElement("model");
	}
	else
	{
		robot_xml = world_xml->FirstChildElement("model");
	}

	// Get all model (robot) elements
	for (; robot_xml; robot_xml = robot_xml->NextSiblingElement("model"))
	{
		UrdfModel *localModel = new UrdfModel;
		m_tmpModels.push_back(localModel);

		XMLElement *stat = robot_xml->FirstChildElement("static");
		if (0 != stat)
		{
			int val = int(atof(stat->GetText()));
			if (val == 1)
			{
				localModel->m_overrideFixedBase = true;
			}
		}

		// Get robot name
		const char *name = robot_xml->Attribute("name");
		if (!name)
		{
			ofLogError() << ("Expected a name for robot");
			return false;
		}
		localModel->m_name = name;

		XMLElement *pose_xml = robot_xml->FirstChildElement("pose");
		if (0 == pose_xml)
		{
			localModel->m_rootTransformInWorld;
		}
		else
		{
			parseTransform(localModel->m_rootTransformInWorld, pose_xml, m_parseSDF);
		}

		// Get all Material elements
		for (XMLElement *material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
		{
			UrdfMaterial *material = new UrdfMaterial;

			parseMaterial(*material, material_xml);

			UrdfMaterial *mat = localModel->m_materials.find(material->m_name)->second;
			if (mat)
			{
				ofLogWarning()<<("Duplicate material");
				delete material;
			}
			else
			{
				localModel->m_materials.insert(std::pair<std::string, UrdfMaterial*>(material->m_name, material));
			}
		}

		for (XMLElement *link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
		{
			UrdfLink *link = new UrdfLink;

			if (parseLink(*localModel, *link, link_xml))
			{
				if (localModel->m_links.find(link->m_name) != localModel->m_links.end())
				{
					ofLogError() << ("Link name is not unique, link names in the same model have to be unique");
					ofLogError() << (link->m_name);
					delete link;
					return false;
				}
				else
				{
					//copy model material into link material, if link has no local material
					for (int i = 0; i < link->m_visualArray.size(); i++)
					{
						UrdfVisual &vis = link->m_visualArray.at(i);
						if (!vis.m_geometry.m_hasLocalMaterial && vis.m_materialName.size() > 0)
						{
							UrdfMaterial *mat = localModel->m_materials.find(vis.m_materialName)->second;
							if (mat)
							{
								vis.m_geometry.m_localMaterial = *mat;
							}
							else
							{
								//ofLogError()<<("Cannot find material with name:");
								//ofLogError()<<(vis.m_materialName);
							}
						}
					}

					localModel->m_links.insert(std::pair<std::string, UrdfLink*>(link->m_name, link));
				}
			}
			else
			{
				ofLogError() << ("failed to parse link");
				delete link;
				return false;
			}
		}
		if (localModel->m_links.size() == 0)
		{
			ofLogWarning()<<("No links found in URDF file");
			return false;
		}

		// Get all Joint elements
		for (XMLElement *joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
		{
			UrdfJoint *joint = new UrdfJoint;

			if (parseJoint(*joint, joint_xml))
			{
				if (localModel->m_joints.find(joint->m_name) != localModel->m_joints.end())
				{
					ofLogError() << ("joint '%s' is not unique.");
					ofLogError() << (joint->m_name);
					delete joint;
					return false;
				}
				else
				{
					localModel->m_joints.insert(std::pair<std::string, UrdfJoint*>(joint->m_name, joint));
				}
			}
			else
			{
				ofLogError() << ("joint xml is not initialized correctly");
				delete joint;
				return false;
			}
		}

		bool ok(initTreeAndRoot(*localModel));
		if (!ok)
		{
			return false;
		}
		m_sdfModels.push_back(localModel);
	}

	return true;
}

std::string UrdfParser::sourceFileLocation(XMLElement *e)
{
#if 0
	//no C++11 etc, no snprintf

	char buf[1024];
	snprintf(buf, sizeof(buf), "%s:%i", m_urdf2Model.m_sourceFile, e->Row());
	return buf;
#else
	char row[1024];
#ifdef G3_TINYXML2
	sprintf(row, "unknown line");
#else
	sprintf(row, "%d", e->GetLineNum());
#endif
	std::string str = m_urdf2Model.m_sourceFile + std::string(":") + std::string(row);
	return str;
#endif
}
