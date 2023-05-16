## **Lucy - Richardson Deconvolution with Matlab and OpenCV**

Lucy-Richardson deconvolution is an iterative algorithm for recovering an image which is blurred by a known point spread function (PSF). You can find the iterative algorithm steps in [Wikipedia](https://en.wikipedia.org/wiki/Richardson–Lucy_deconvolution).

Recently, I needed to use it in OpenCV. However, I realised that there is not a built-in function in OpenCV libraries for this deconvolution method. So I decided to post my quick code in OpenCV. Before that, let's analyse the method in Matlab on a sample image.

Below, there is a sample image of LENA text.

[![img](https://1.bp.blogspot.com/-fi1eD-5nScY/WLXM6Y3I9yI/AAAAAAAAAT8/tl2p14uASp40yJDLec83e8EdV5dRLH2cQCLcB/s320/lena_words.png)](https://1.bp.blogspot.com/-fi1eD-5nScY/WLXM6Y3I9yI/AAAAAAAAAT8/tl2p14uASp40yJDLec83e8EdV5dRLH2cQCLcB/s1600/lena_words.png)

If we blur it with a PSF, here is the result:

[![img](https://4.bp.blogspot.com/-WBwWWzAmQX0/WLXNSK73bcI/AAAAAAAAAUA/Y2O-2iiEceMMRVEA3FoCxETudnw1lxCjgCLcB/s320/lena_words_blurred.png)](https://4.bp.blogspot.com/-WBwWWzAmQX0/WLXNSK73bcI/AAAAAAAAAUA/Y2O-2iiEceMMRVEA3FoCxETudnw1lxCjgCLcB/s1600/lena_words_blurred.png)

After Lucy-Richardson deconvolution with the same PSF with 100 iterations, I obtained the image below:

[![img](https://4.bp.blogspot.com/-xrtgsI1_Ei0/WLXNqYRp5ZI/AAAAAAAAAUE/ZY-K1bMHYg8V9fTLpHJnQMTanT12HtRmwCLcB/s320/lena_words_deconv.png)](https://4.bp.blogspot.com/-xrtgsI1_Ei0/WLXNqYRp5ZI/AAAAAAAAAUE/ZY-K1bMHYg8V9fTLpHJnQMTanT12HtRmwCLcB/s1600/lena_words_deconv.png)

Here is the Matlab code:

[![img](https://1.bp.blogspot.com/-pn4mNxmgJF8/WLh0784DsTI/AAAAAAAAAW0/FlEqxZs39q4OgYOFbXztegmBzQG65oNvgCLcB/s400/lucy_matlab_1.png)](https://1.bp.blogspot.com/-pn4mNxmgJF8/WLh0784DsTI/AAAAAAAAAW0/FlEqxZs39q4OgYOFbXztegmBzQG65oNvgCLcB/s1600/lucy_matlab_1.png)

Actually, you can see the source code for some of the functions in Matlab by typing "open <function name>" or "edit <function name>". So, if you try opening "deconvlucy.m" function you can see the steps of the function. Below, I tried to simplify that code. 

[![img](https://1.bp.blogspot.com/-Mj5DdgeJesY/WLh0tP-ut0I/AAAAAAAAAWw/w5h42MsrbbArjkd6mcWRKbR_xwh8tyHngCLcB/s640/lucy_matlab_2.png)](https://1.bp.blogspot.com/-Mj5DdgeJesY/WLh0tP-ut0I/AAAAAAAAAWw/w5h42MsrbbArjkd6mcWRKbR_xwh8tyHngCLcB/s1600/lucy_matlab_2.png)

After running this code, I managed to get the same output of the original "deconvlucy.m" function.



[![img](https://3.bp.blogspot.com/-g61L7f4zGqs/WLh1SuRgFvI/AAAAAAAAAW4/x8D-HSuQrYIfx2tbk8Rt1qdyM0zmiCTOwCLcB/s320/lena_words_deconv.png)](https://3.bp.blogspot.com/-g61L7f4zGqs/WLh1SuRgFvI/AAAAAAAAAW4/x8D-HSuQrYIfx2tbk8Rt1qdyM0zmiCTOwCLcB/s1600/lena_words_deconv.png)

You can access the Matlab code [here](https://drive.google.com/open?id=0B7CO_0Z4IxeRbW5TNzRic19TZ3M). You can also access the same image that is used in the code [here](https://drive.google.com/open?id=0B7CO_0Z4IxeRRC1LRW9STkhrX2s).

Now let's do the same in OpenCV.



[![img](https://1.bp.blogspot.com/-yOA3eKFen54/WLs03AgGt1I/AAAAAAAAAXI/_sDaq7rVxBkYwDyZwbs0gb5vZLiCEF9EACLcB/s640/opencv_code.png)](https://1.bp.blogspot.com/-yOA3eKFen54/WLs03AgGt1I/AAAAAAAAAXI/_sDaq7rVxBkYwDyZwbs0gb5vZLiCEF9EACLcB/s1600/opencv_code.png)

After running this code, I was able to get the same results:



[![img](https://4.bp.blogspot.com/-wKXHgCR6buU/WLxfGNEWsjI/AAAAAAAAAXo/7wlkK-TdABYfcGV_uZYBps0d-BuCS0W2gCLcB/s640/lena_opencv.png)](https://4.bp.blogspot.com/-wKXHgCR6buU/WLxfGNEWsjI/AAAAAAAAAXo/7wlkK-TdABYfcGV_uZYBps0d-BuCS0W2gCLcB/s1600/lena_opencv.png)

Here is another example: 

[![img](https://1.bp.blogspot.com/-i3UM7uIHOTE/WLxenUQBBdI/AAAAAAAAAXg/ibttIt1BYDUBMn3gTJi7GVrtAW0MbTu5wCLcB/s640/stars_opencv.png)](https://1.bp.blogspot.com/-i3UM7uIHOTE/WLxenUQBBdI/AAAAAAAAAXg/ibttIt1BYDUBMn3gTJi7GVrtAW0MbTu5wCLcB/s1600/stars_opencv.png)

You can access the whole c++ code [here](https://drive.google.com/open?id=0B7CO_0Z4IxeRWVdXSjZFRzF3Y0k). You can also access the same image that is used in the code [here](https://drive.google.com/open?id=0B7CO_0Z4IxeRc05scWE1WjI0TXM). If you have any questions, please comment below.





Posted 1st March 2017 by [Abdullah H. Özcan](http://www.blogger.com/profile/03426151922830576095)

 