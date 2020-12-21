//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
//== INCLUDES =================================================================

#include <graphene/gl/Texture.h>
#include <stb_image/stb_image.h>
#include <stb_image/stb_image_write.h>
#include <stb_image/tiny_jpeg.h>
#include <cstring>

//== NAMESPACE ================================================================

namespace graphene {
namespace gl {
//=============================================================================


Texture::Texture()  :
    id_(0),
    width_(0),
    height_(0),
    channels_(4),
    data_(0),
    srgb_(true),
    filter_(GL_LINEAR)
{

}

Texture::Texture(const std::string &name, int width, int height, unsigned char *data) :
    id_(0),
    name_(name),
    width_(width),
    height_(height),
    channels_(4),
    data_(0),
    srgb_(true),
    filter_(GL_LINEAR)
{
    data_.resize(width_*height_*channels_);
    memcpy(data_.data(), data, width_*height_*channels_*sizeof(unsigned char));
}

Texture::~Texture()
{
    if (id_) glDeleteTextures(1, &id_);
}

void Texture::deep_copy(Texture& out_texture)
{
    out_texture.id_ = 0;
    out_texture.name_ = this->name_;
    out_texture.width_ = this->width_;
    out_texture.height_ = this->height_;
    out_texture.channels_ = this->channels_;

    out_texture.data_ = this->data_;

    out_texture.srgb_ = this->srgb_;
    out_texture.filter_ = this->filter_;
}

bool Texture::read(const std::string &filename)
{
    int channels_ignored;

    stbi_set_flip_vertically_on_load(1);
    stbi_uc *data = stbi_load(filename.c_str(), &width_, &height_, &channels_ignored, channels_);

    if (data == nullptr)
    {
        std::cerr << "Texture::read: [ERROR] Could not read texture with filename: \"" << filename << "\"." << std::endl;
        return false;
    }
    name_ = filename;
    data_.resize(width_*height_*channels_);
    memcpy(data_.data(), data, width_*height_*channels_*sizeof(unsigned char));

    return true;
}

bool Texture::write(const std::string &filename)
{

//    size_t dot = filename.rfind('.');
//    if (! (filename.substr(dot+1) == "png") )
//    {
//        std::cout << "Texture::write: [ERROR] Only writing of png files is supported." << std::endl;
//        return false;
//    }
    
//    stbi_flip_vertically_on_write(1);
//    const int result = stbi_write_png(filename.c_str(), width_, height_, channels_, data_, sizeof(unsigned char) * 4 * width_);

    //create y-flipped texture
    Texture* flipped = flip_y();
    //write flipped texture
    const int result = tje_encode_to_file(filename.c_str(), flipped->width_, flipped->height_, flipped->channels_, flipped->data_.data());
    //delete flipped
    delete flipped;

    return (result > 0);
}


Texture* Texture::flip_y()
{

    Texture* out = new Texture("temp", width_, height_, new unsigned char[width_*height_*channels_]);

    for (int i=0; i < width_; ++i)
    {
        for (int j=0; j < height_; ++j)
        {
            texel(out, i, j, 0) = texel(this, i, (height_-1) - j , 0);
            texel(out, i, j, 1) = texel(this, i, (height_-1) - j , 1);
            texel(out, i, j, 2) = texel(this, i, (height_-1) - j , 2);
        }
    }


    return out;
}

unsigned char& texel(Texture* t, int x, int y, int ch)
{
    return t->data_[y*(t->width_*t->channels_) + (x*t->channels_)+ch];
}


//=============================================================================
} // namespace gl
} // namespace graphene
//=============================================================================
