#include "mmpch.h"
#include "nnet.h"


void nnet_load(nnet& nn, const char* filename)
{
	FILE* f = fopen(filename, "rb");
	assert(f != NULL);

	array1d_read(nn.input_mean, f);
	array1d_read(nn.input_std, f);
	array1d_read(nn.output_mean, f);
	array1d_read(nn.output_std, f);

	int count;
	fread(&count, sizeof(int), 1, f);

	nn.weights.resize(count);
	nn.biases.resize(count);

	for (int i = 0; i < count; i++)
	{
		array2d_read(nn.weights[i], f);
		array1d_read(nn.biases[i], f);
	}

	fclose(f);
}


// Neural Network evaluation function. Assumes input 
// has been placed in first layer of `evaulation` 
// object. Puts result in the last layer of the 
// `evaluation` object.
void nnet_evaluate(
	nnet_evaluation& evaluation,
	const nnet& nn)
{
	nnet_layer_normalize(
		evaluation.layers.front(),
		nn.input_mean,
		nn.input_std);

	for (int i = 0; i < nn.weights.size(); i++)
	{
		nnet_layer_linear(
			evaluation.layers[i + 1],
			evaluation.layers[i],
			nn.weights[i],
			nn.biases[i]);

		// No relu for final layer
		if (i != nn.weights.size() - 1)
		{
			nnet_layer_relu(evaluation.layers[i + 1]);
		}
	}

	nnet_layer_denormalize(
		evaluation.layers.back(),
		nn.output_mean,
		nn.output_std);
}

