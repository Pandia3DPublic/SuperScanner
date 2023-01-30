#pragma once
#include <sstream>

class Shader
{
public:
    unsigned int ID;
    // constructor generates the shader on the fly
    // ------------------------------------------------------------------------
    Shader(const char* vertexPath, const char* fragmentPath)
    {
        // 1. retrieve the vertex/fragment source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        // ensure ifstream objects can throw exceptions:
        vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        try 
        {
            // open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            std::stringstream vShaderStream, fShaderStream;
            // read file's buffer contents into streams
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            // close file handlers
            vShaderFile.close();
            fShaderFile.close();
            // convert stream into string
            vertexCode   = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }
        const char* vShaderCode = vertexCode.c_str();
        const char * fShaderCode = fragmentCode.c_str();
        // 2. compile shaders
        unsigned int vertex, fragment;
        // vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        // fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        // shader Program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
        // delete the shaders as they're linked into our program now and no longer necessary
        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    Shader(const char* vertexPath, const char* fragmentPath, const char* geometryPath)
    {
        // opengl shader pipeline: vertex -> (geometry) -> fragment
        // 1. retrieve the vertex/fragment/geometry source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::string geometryCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        std::ifstream gShaderFile;
        // ensure ifstream objects can throw exceptions:
        vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        gShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        try 
        {
            // open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            gShaderFile.open(geometryPath);
            std::stringstream vShaderStream, fShaderStream, gShaderStream;
            // read file's buffer contents into streams
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            gShaderStream << gShaderFile.rdbuf();
            // close file handlers
            vShaderFile.close();
            fShaderFile.close();
            gShaderFile.close();
            // convert stream into string
            vertexCode   = vShaderStream.str();
            fragmentCode = fShaderStream.str();
            geometryCode   = gShaderStream.str();
        }
        catch (std::ifstream::failure e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }
        const char* vShaderCode = vertexCode.c_str();
        const char * fShaderCode = fragmentCode.c_str();
        const char* gShaderCode = geometryCode.c_str();
        // 2. compile shaders
        unsigned int vertex, fragment, geometry;
        // vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        // fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        // geometry shader
        geometry = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry, 1, &gShaderCode, NULL);
        glCompileShader(geometry);
        checkCompileErrors(geometry, "GEOMETRY");
        // shader Program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glAttachShader(ID, geometry);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
        // delete the shaders as they're linked into our program now and no longer necessary
        glDeleteShader(vertex);
        glDeleteShader(fragment);
        glDeleteShader(geometry);
    }

    // activate the shader
    // ------------------------------------------------------------------------
    void use() 
    { 
        glUseProgram(ID); 
    }
    // utility uniform functions
    // ------------------------------------------------------------------------
    void setBool(const std::string &name, bool value) const
    {         
       // glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value); 
		GLuint loc = glGetUniformLocation(ID, name.c_str());
		glProgramUniform1i(ID, loc,(int)value);
    }
    // ------------------------------------------------------------------------
    void setInt(const std::string &name, int value) const
    { 
        //glUniform1i(glGetUniformLocation(ID, name.c_str()), value); 
		GLuint loc = glGetUniformLocation(ID, name.c_str());
		glProgramUniform1i(ID, loc,(int)value);
    }
    // ------------------------------------------------------------------------
    void setFloat(const std::string &name, float value) const
    { 
		GLuint loc = glGetUniformLocation(ID, name.c_str());
		glProgramUniform1f(ID, loc,value);

        //glUniform1f(glGetUniformLocation(ID, name.c_str()), value); 
    }

	void setVec3(const std::string &name, const Eigen::Vector3f& v) const
	{ 
		GLuint loc = glGetUniformLocation(ID, name.c_str());
		//glUniform3f(glGetUniformLocation(ID, name.c_str()), (float) v(0), (float) v(1),(float) v(2)); 
		glProgramUniform3fv(ID, loc,1,v.data());

	}
	void setUniform4(const std::string &name,const Eigen::Vector4f& v) const
	{ 
		GLuint loc = glGetUniformLocation(ID, name.c_str());
		//glUniform4f(glGetUniformLocation(ID, name.c_str()),(float) v(0), (float) v(1),(float) v(2),(float) v(3)); 
		glProgramUniform4fv(ID, loc,1,v.data());
	}
	void setUniform4Matrix(const GLuint& loc, const Eigen::Matrix4f& m) const
	{ 
		glProgramUniformMatrix4fv(ID,loc,1,false,m.data());

	}
	void setUniform4Matrix(const std::string &name, const Eigen::Matrix4f& m) const
	{ 
		GLuint loc = glGetUniformLocation(ID, name.c_str());
		glProgramUniformMatrix4fv(ID,loc,1,false,m.data());

	}

	GLuint getAttribLocation(const std::string& s) {
		return glGetAttribLocation(ID, s.c_str());
	}

	GLuint getUniformLocation(const std::string& s) {
		return glGetUniformLocation(ID, s.c_str());
	}

private:
    // utility function for checking shader compilation/linking errors.
    // ------------------------------------------------------------------------
    void checkCompileErrors(unsigned int shader, std::string type)
    {
        int success;
        char infoLog[1024];
        if (type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};
