---
layout:     post
title:      "Bag of Words"
date:       2018-10-26
author:     Tong
catalog: true
tags:
    - SLAM
---

## 1. [Video Google: A Text Retrieval Approach to Object Matching in Videos][paper-Bag-of-Words]

1. _What is the objective of the approach?_
<br> An approach to object and scene retrieval which searches for and localizes all the occurrences of a user outlined object in a video.

2. _How is the object represented?_
<br> The object is represented by a set of viewpoint invariant region descriptors so that recognition can proceed successfully despite changes in viewpoint, illumination and partial occlusion.

3. _What are the benefits of the approach?_
<br> The benefits of this approach is that matches are effectively pre-computed so that at run-time frames and shots containing any particular object can be retrieved with nodelay.

4. _What is an inverted file?_
<br> The set of vectors representing all the documents in a corpus are organized as an _inverted file_ to facilitate efficient retrieval. An inverted file is structured like an ideal book index. It has an entry for each word in the corpus followed by a list of all the documents (and position in that document) in which the word occurs.

5. _Which types of view point covariant regions are computed for each frame?_
<br> - Shape Adapted (SA)
<br> - Maximally Stable (MS)

6. _How is the region represented?_
<br> Each elliptical affine invariant region is represented by a 128-dimensional vector using the SIFT descriptor.

7. _How to reduce noise and reject unstable regions?_
<br> Information is aggregated over a sequence of frames. Any region which does not survive for more than three frames is rejected.

8. _Why building a visual vocabulary?_
<br> The objective here is to vector quantize the descriptors into clusters which will be the visual 'words' for text retrieval. Then when a new frame is observed each descriptor of the frame is assigned to the nearest cluster, and this immediately generates matches for all frames.

9. _How is the vector quantization done?_
<br> By [K-means clustering][website-k-means], though other methods (K-medoids, histogram binning, etc) are possible.

10. _How is each document represented in text retrieval?_
<br> It is represented  by a vector of word frequencies. However, it is usual to apply a weighting to the components of this vector according to [Reference][book-modern-information-retrieval]. 

11. _What is a stop list?_
<br> Using a stop list analogy the most frequent visual words that occur in almost all images are suppressed. The stop list boundaries were determined empirically to reduce the number of mismatches and size of the inverted file while keeping sufficient visual vocabulary.

12. Matches can also be filterd on _spatial consistency_, which means that matched covariant regions in the retrieved frames should have a similar spatial arrrangement to those of the outlined region in the query image.

13. _What is an inverted file?_
<br> In a classcical file structure all words are stored in the document they appear in. An inverted file structure has an entry (hit list) for each word where all occurrences of the word in all documents are stored. In our case the inverted file has an entry for each visual word, which stores all the matches, i.e. occurrences of the same word in all frames. The document vector is very sparse and use of an inverted file makes the retrieval very fast.
<br> You can find more details about an inverted file in this [website][website-inverted-file].

14. Summary: They perform retrieval of shots from a movie using a text retrieval approach. Descriptors extracted from local affine invariant regions are quantized into visual words, which are defined by _k_-means performed on the descriptor vectors from a number of training frames. The collection of visual words are used in Term Frequency Inverse Document Frequency ([TF-IDF][wiki-tf-idf]) scoring of the relevance of an image to the query. The scoring is accomplished using inverted files.

<br>
<br>
## 2. [Scalable Recognition with a Vocabulary Tree][paper-vocabulary-tree]

1. The most significant property of the scheme is that the tree directly defines the quantization. The quantization and the indexing are therefore fully integrated, essentially being one and the same.

2. The approach belongs to a currently very popular class of algorithms that work with local image regions and represent an object with descriptors extracted from these local regions. The strength of this class of algorithms is natural robustness against occlusion and background clutter.

3. The most important contribution of this paper is an indexing mechanism that enables extremely efficient retrieval. We propose a hierarchical TF-IDF scoring using hierarchiacally defined visual words that form a vocabulary tree. This allows much more efficient lookup of visual words.

4. We use hierarchical scoring, meaning that other nodes than the leaf nodes are considered, but the number of images attached to the inverted file of a node are limited to a fixed number, since larger inverted files are expensive and provide little entropy in the TF-IDF scoring.

5. We can insert images into the database at the same rate as reported for the feature extraction, i.e. around 5Hz for 640 x 480 resolution. This potential for on-the-fly insertion of new objects into the database is a result of the quantization into visual words, which is defined once and for all, while still allowing general high retrieval performance.

6. We use proximity of the descriptor vectors to various cluster centers defining the vocabulary tree. We use an offline unsupervised training stage to define the vocabulary tree, but once the vocabulary tree is determined, new images can be inserted on-the-fly into the database.

7. The compactness of the database is very important for query efficiency in a large database. With our vocabulary tree approach, the representation of an image is simply one or two integers, which should be contrasted to the hundreds of bytes or floats used for a descriptor vector.

8. For feature extraction, we use our own implementation of Maximally Stable Extremal Regions ([MSERs][paper-mser]). We warp an elliptical patch around each MSER region into a circular patch. The remaining portion of our feature extraction is then implemented according the [SIFT][paper-sift] feature extraction pipeline. Canonical directions are found based on an orientation histogram formed on the image gradients. SIFT descriptors are then extracted relative to the canonical directions. The normalized SIFT descriptors are then quantized with the vocabulary tree. Finally, a hierarchical scoring scheme is applied to retrieve images from a database.

9. 

[paper-Bag-of-Words]: http://www.robots.ox.ac.uk/~vgg/publications/papers/sivic03.pdf
[paper-vocabulary-tree]: http://www-inst.eecs.berkeley.edu/~cs294-6/fa06/papers/nister_stewenius_cvpr2006.pdf
[website-k-means]: http://lingtong.de/2018/10/26/K-Means-Clustering/
[book-modern-information-retrieval]: http://people.ischool.berkeley.edu/~hearst/irbook/print/chap10.pdf
[website-inverted-file]: http://orion.lcg.ufrj.br/Dr.Dobbs/books/book5/chap03.htm
[wiki-tf-idf]: https://zh.wikipedia.org/wiki/Tf-idf
[paper-mser]: http://cmp.felk.cvut.cz/~matas/papers/matas-bmvc02.pdf
[paper-sift]: https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf