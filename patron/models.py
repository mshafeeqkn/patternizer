from django.db import models


class Image(models.Model):
    name = models.CharField(max_length=50)
    image = models.ImageField(upload_to='images/')
    paper_size = models.CharField(max_length=50, default='')
    num_img = models.CharField(max_length=50, default='')
    labels = models.CharField(max_length=500, default='')