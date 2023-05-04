FFmpeg README
=============

FFmpeg is a collection of libraries and tools to process multimedia content
such as audio, video, subtitles and related metadata.

## Compiling the customizaed FFmpeg for our experiment

- Linux:

First install all the dependencies following the [official compilation guide](https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu)

Then run the following command in the source code folder of this project:
```
PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig" ./configure \
  --prefix="$HOME/ffmpeg_build" \
  --pkg-config-flags="--static" \
  --extra-cflags="-I$HOME/ffmpeg_build/include -static -Wno-error=vla" \
  --extra-ldflags="-L$HOME/ffmpeg_build/lib -static" \
  --extra-libs="-lpthread -lm -lgsl -lgslcblas" \
  --ld="g++" \
  --bindir="$HOME/bin" \
  --enable-gpl \
  --enable-libx264 \
  --enable-nonfree \
  --enable-libfreetype \
  --enable-libfontconfig \
  --enable-filter=watermark && \
PATH="$HOME/bin:$PATH" make && \
make install
```

## License

FFmpeg codebase is mainly LGPL-licensed with optional components licensed under
GPL. Please refer to the LICENSE file for detailed information.
