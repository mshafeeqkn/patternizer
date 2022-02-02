from django.db import models


class Image(models.Model):
    name = models.CharField(max_length=50)
    image = models.ImageField(upload_to='images/')
    data_file = models.FileField(upload_to='data/')


class LFont(models.Model):
    font = models.FileField(upload_to='fonts/')
